/*!
 * @file Adafruit_BMP3XX.cpp
 *
 * @mainpage Adafruit BMP3XX temperature & barometric pressure sensor driver
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's BMP3XX driver for the
 * Arduino platform.  It is designed specifically to work with the
 * Adafruit BMP388 breakout: https://www.adafruit.com/products/3966
 *
 * These sensors use I2C or SPI to communicate, 2 or 4 pins are required
 * to interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Ladyada for Adafruit Industries.
 * Modified to use direct SPI (no Adafruit_BusIO dependency).
 *
 * @section license License
 *
 * BSD license, all text above must be included in any redistribution
 *
 */

#include "Adafruit_BMP3XX.h"
#include "Arduino.h"

// Global SPI context used by transport callbacks
static uint8_t g_cs_pin;
static uint32_t g_spi_freq;
static SPIClass *g_spi;

// Transport function prototypes
static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr);
static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len,
                        void *intf_ptr);
static void delay_usec(uint32_t us, void *intf_ptr);
static int8_t validate_trimming_param(struct bmp3_dev *dev);
static int8_t cal_crc(uint8_t seed, uint8_t data);

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

Adafruit_BMP3XX::Adafruit_BMP3XX(void) {
  _filterEnabled = _tempOSEnabled = _presOSEnabled = _ODREnabled = false;
  _cs = -1;
}

/*!
 *    @brief  Sets up the hardware and initializes hardware SPI
 *    @param  cs_pin The arduino pin # connected to chip select
 *    @param  theSPI The SPI object to be used for SPI connections.
 *    @param  frequency The SPI bus frequency
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_BMP3XX::begin_SPI(uint8_t cs_pin, SPIClass *theSPI,
                                uint32_t frequency) {
  _cs = cs_pin;

  // Store globals for transport callbacks
  g_cs_pin = cs_pin;
  g_spi_freq = frequency;
  g_spi = theSPI;

  // Configure CS pin
  pinMode(_cs, OUTPUT);
  digitalWrite(_cs, HIGH);

  // Initialize SPI bus
  theSPI->begin();

  the_sensor.chip_id = cs_pin;
  the_sensor.intf = BMP3_SPI_INTF;
  the_sensor.read = &spi_read;
  the_sensor.write = &spi_write;
  the_sensor.intf_ptr = theSPI;  // must be non-NULL for Bosch driver null check
  the_sensor.dummy_byte = 1;

  return _init();
}

bool Adafruit_BMP3XX::_init(void) {
  the_sensor.delay_us = delay_usec;
  int8_t rslt = BMP3_OK;

  /* Reset the sensor */
  rslt = bmp3_soft_reset(&the_sensor);
  if (rslt != BMP3_OK) {
    Serial.print("BMP3XX: soft_reset failed, rslt=");
    Serial.println(rslt);
    return false;
  }

  rslt = bmp3_init(&the_sensor);
  if (rslt != BMP3_OK) {
    Serial.print("BMP3XX: bmp3_init failed, rslt=");
    Serial.println(rslt);
    return false;
  }

  rslt = validate_trimming_param(&the_sensor);
  if (rslt != BMP3_OK) {
    Serial.print("BMP3XX: trimming CRC failed, rslt=");
    Serial.println(rslt);
    return false;
  }

  setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  setPressureOversampling(BMP3_NO_OVERSAMPLING);
  setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  setOutputDataRate(BMP3_ODR_25_HZ);

  // don't do anything till we request a reading
  the_sensor.settings.op_mode = BMP3_MODE_FORCED;

  return true;
}

float Adafruit_BMP3XX::readTemperature(void) {
  performReading();
  return temperature;
}

uint8_t Adafruit_BMP3XX::chipID(void) { return the_sensor.chip_id; }

float Adafruit_BMP3XX::readPressure(void) {
  performReading();
  return pressure;
}

float Adafruit_BMP3XX::readAltitude(float seaLevel) {
  float atmospheric = readPressure() / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));
}

bool Adafruit_BMP3XX::performReading(void) {
  int8_t rslt;
  uint16_t settings_sel = 0;
  uint8_t sensor_comp = 0;

  the_sensor.settings.temp_en = BMP3_ENABLE;
  settings_sel |= BMP3_SEL_TEMP_EN;
  sensor_comp |= BMP3_TEMP;
  if (_tempOSEnabled) {
    settings_sel |= BMP3_SEL_TEMP_OS;
  }

  the_sensor.settings.press_en = BMP3_ENABLE;
  settings_sel |= BMP3_SEL_PRESS_EN;
  sensor_comp |= BMP3_PRESS;
  if (_presOSEnabled) {
    settings_sel |= BMP3_SEL_PRESS_OS;
  }

  if (_filterEnabled) {
    settings_sel |= BMP3_SEL_IIR_FILTER;
  }

  if (_ODREnabled) {
    settings_sel |= BMP3_SEL_ODR;
  }

  rslt = bmp3_set_sensor_settings(settings_sel, &the_sensor);
  if (rslt != BMP3_OK)
    return false;

  the_sensor.settings.op_mode = BMP3_MODE_FORCED;
  rslt = bmp3_set_op_mode(&the_sensor);
  if (rslt != BMP3_OK)
    return false;

  struct bmp3_data data;

  rslt = bmp3_get_sensor_data(sensor_comp, &data, &the_sensor);
  if (rslt != BMP3_OK)
    return false;

  temperature = data.temperature;
  pressure = data.pressure;

  return true;
}

bool Adafruit_BMP3XX::setTemperatureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X)
    return false;

  the_sensor.settings.odr_filter.temp_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _tempOSEnabled = false;
  else
    _tempOSEnabled = true;

  return true;
}

bool Adafruit_BMP3XX::setPressureOversampling(uint8_t oversample) {
  if (oversample > BMP3_OVERSAMPLING_32X)
    return false;

  the_sensor.settings.odr_filter.press_os = oversample;

  if (oversample == BMP3_NO_OVERSAMPLING)
    _presOSEnabled = false;
  else
    _presOSEnabled = true;

  return true;
}

bool Adafruit_BMP3XX::setIIRFilterCoeff(uint8_t filtercoeff) {
  if (filtercoeff > BMP3_IIR_FILTER_COEFF_127)
    return false;

  the_sensor.settings.odr_filter.iir_filter = filtercoeff;

  if (filtercoeff == BMP3_IIR_FILTER_DISABLE)
    _filterEnabled = false;
  else
    _filterEnabled = true;

  return true;
}

bool Adafruit_BMP3XX::setOutputDataRate(uint8_t odr) {
  if (odr > BMP3_ODR_0_001_HZ)
    return false;

  the_sensor.settings.odr_filter.odr = odr;

  _ODREnabled = true;

  return true;
}

/***************************************************************************
 TRANSPORT FUNCTIONS (direct SPI, no Adafruit_BusIO)
 ***************************************************************************/

static int8_t spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len,
                       void *intf_ptr) {
  (void)intf_ptr;
  g_spi->beginTransaction(SPISettings(g_spi_freq, MSBFIRST, SPI_MODE0));
  digitalWrite(g_cs_pin, LOW);
  g_spi->transfer(reg_addr);
  for (uint32_t i = 0; i < len; i++) {
    reg_data[i] = g_spi->transfer(0xFF);
  }
  digitalWrite(g_cs_pin, HIGH);
  g_spi->endTransaction();
  return 0;
}

static int8_t spi_write(uint8_t reg_addr, const uint8_t *reg_data,
                        uint32_t len, void *intf_ptr) {
  (void)intf_ptr;
  g_spi->beginTransaction(SPISettings(g_spi_freq, MSBFIRST, SPI_MODE0));
  digitalWrite(g_cs_pin, LOW);
  g_spi->transfer(reg_addr);
  for (uint32_t i = 0; i < len; i++) {
    g_spi->transfer(reg_data[i]);
  }
  digitalWrite(g_cs_pin, HIGH);
  g_spi->endTransaction();
  return 0;
}

static void delay_usec(uint32_t us, void *intf_ptr) {
  (void)intf_ptr;
  delayMicroseconds(us);
}

static int8_t validate_trimming_param(struct bmp3_dev *dev) {
  int8_t rslt;
  uint8_t crc = 0xFF;
  uint8_t stored_crc;
  uint8_t trim_param[21];
  uint8_t i;

  rslt = bmp3_get_regs(BMP3_REG_CALIB_DATA, trim_param, 21, dev);
  if (rslt == BMP3_OK) {
    for (i = 0; i < 21; i++) {
      crc = (uint8_t)cal_crc(crc, trim_param[i]);
    }

    crc = (crc ^ 0xFF);
    rslt = bmp3_get_regs(0x30, &stored_crc, 1, dev);
    if (stored_crc != crc) {
      rslt = -1;
    }
  }

  return rslt;
}

static int8_t cal_crc(uint8_t seed, uint8_t data) {
  int8_t poly = 0x1D;
  int8_t var2;
  uint8_t i;

  for (i = 0; i < 8; i++) {
    if ((seed & 0x80) ^ (data & 0x80)) {
      var2 = 1;
    } else {
      var2 = 0;
    }

    seed = (seed & 0x7F) << 1;
    data = (data & 0x7F) << 1;
    seed = seed ^ (uint8_t)(poly * var2);
  }

  return (int8_t)seed;
}
