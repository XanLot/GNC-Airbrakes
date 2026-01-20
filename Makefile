# GNC-Airbrakes Teensy 4.1 Firmware Makefile
# Based on reference Makefile by Jack Miller 2024 (Github: guywithhat99)

# Detect the current operating system using uname
UNAME := $(shell uname -s)

# The name of the target executable
TARGET_EXEC := firmware

# Directory where build outputs will be placed
BUILD_DIR := ./build

# Tools directory
TOOLS_DIR := ./tools

# Source directories
TEENSY_SRC_DIRS := ./teensy4
LIBRARY_SRC_DIRS := ./libraries
SRC_SRC_DIRS := ./src

# Find all C, C++, and assembly source files in the specified directories
TEENSY_SRC := $(shell find $(TEENSY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
LIBRARY_SRC := $(shell find $(LIBRARY_SRC_DIRS) -name '*.cpp' -or -name '*.c')
SRC_SRC := $(shell find $(SRC_SRC_DIRS) -name '*.cpp' -or -name '*.c')

# Generate object file paths by prepending BUILD_DIR and appending .o to source files
TEENSY_OBJS := $(TEENSY_SRC:%=$(BUILD_DIR)/%.o)
LIBRARY_OBJS := $(LIBRARY_SRC:%=$(BUILD_DIR)/%.o)
SRC_OBJS := $(SRC_SRC:%=$(BUILD_DIR)/%.o)

# Generate dependency file paths by replacing .o with .d in object file paths
TEENSY_DEPS := $(TEENSY_OBJS:.o=.d)
LIBRARY_DEPS := $(LIBRARY_OBJS:.o=.d)
SRC_DEPS := $(SRC_OBJS:.o=.d)

# Find all include directories for GCC to locate header files
TEENSY_INC_DIRS := $(shell find $(TEENSY_SRC_DIRS) -type d)
LIBRARY_INC_DIRS := $(shell find $(LIBRARY_SRC_DIRS) -maxdepth 2 -type d)
SRC_INC_DIRS := $(shell find $(SRC_SRC_DIRS) -type d)

# Generate compiler include flags from include directories
# -isystem on Teensy and Library files to suppress warnings
TEENSY_INC_FLAGS := $(addprefix -isystem,$(TEENSY_INC_DIRS))
LIBRARY_INC_FLAGS := $(addprefix -isystem,$(LIBRARY_INC_DIRS))
SRC_INC_FLAGS := $(addprefix -I,$(SRC_INC_DIRS))
INCLUDE_FLAGS := $(TEENSY_INC_FLAGS) $(LIBRARY_INC_FLAGS) $(SRC_INC_FLAGS)

# Compiler flags specific to Teensy 4.1
TEENSY4_FLAGS = -DF_CPU=600000000 -DUSB_SERIAL -DLAYOUT_US_ENGLISH -D__IMXRT1062__ -DTEENSYDUINO=159 -DARDUINO_TEENSY41 -DARDUINO=10813

# CPU flags to optimize code for the Teensy processor
CPU_CFLAGS = -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 -mthumb

DEFINES := $(TEENSY4_FLAGS)

# Preprocessor flags for both C and C++ files
CPPFLAGS := $(INCLUDE_FLAGS) $(DEFINES) -MMD -MP -ffunction-sections -fdata-sections -O2 --specs=nano.specs -g3

# Compiler flags for C files
CFLAGS := $(CPU_CFLAGS)

# Compiler flags for C++ files
CXXFLAGS := $(CPU_CFLAGS) -std=gnu++23 \
            -felide-constructors -fno-exceptions -fpermissive -fno-rtti \
            -Wno-error=narrowing -Wno-trigraphs -Wno-comment -Wall -Werror \
            -Wno-volatile

# Linker flags, including Teensy-specific linker script
LINKING_FLAGS = -Wl,--gc-sections,--relax,-Tteensy4/imxrt1062_t41.ld,--print-memory-usage,-Map=$(BUILD_DIR)/$(TARGET_EXEC).map,--cref

# Set the Arduino path based on the detected operating system
ifeq ($(UNAME),Darwin)
 ARDUINO_PATH = $(abspath $(HOME)/Library/Arduino15)
 $(info Detected macOS)
endif
ifeq ($(UNAME),Linux)
 ARDUINO_PATH = $(abspath $(HOME)/.arduino15)
 $(info Detected Linux)
endif

# Base arm-none-eabi tool paths
COMPILER_TOOLS_PATH = $(TOOLS_DIR)/compiler/arm-gnu-toolchain/bin

# arm-none-eabi tools
COMPILER_CPP = $(COMPILER_TOOLS_PATH)/arm-none-eabi-g++
COMPILER_C   = $(COMPILER_TOOLS_PATH)/arm-none-eabi-gcc
AR           = $(COMPILER_TOOLS_PATH)/arm-none-eabi-ar
GDB          = $(COMPILER_TOOLS_PATH)/arm-none-eabi-gdb
OBJCOPY      = $(COMPILER_TOOLS_PATH)/arm-none-eabi-objcopy
OBJDUMP      = $(COMPILER_TOOLS_PATH)/arm-none-eabi-objdump
READELF      = $(COMPILER_TOOLS_PATH)/arm-none-eabi-readelf
ADDR2LINE    = $(COMPILER_TOOLS_PATH)/arm-none-eabi-addr2line
SIZE         = $(COMPILER_TOOLS_PATH)/arm-none-eabi-size

# Utilize all available CPU cores for parallel build
MAKEFLAGS += -j$(nproc)

# Phony targets
.PHONY: build clean clean_src clean_libs clean_teensy4 upload install help cdb

# Main build target
build: $(BUILD_DIR)/$(TARGET_EXEC)

# Final linking step to create the executable
$(BUILD_DIR)/$(TARGET_EXEC): $(SRC_OBJS) $(LIBRARY_OBJS) $(TEENSY_OBJS)
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) $(LIBRARY_OBJS) $(TEENSY_OBJS) $(SRC_OBJS) $(LINKING_FLAGS) -o $(BUILD_DIR)/$(TARGET_EXEC).elf
	@cp $(BUILD_DIR)/$(TARGET_EXEC).elf .
	@echo [Constructing $(TARGET_EXEC).hex]
	@$(OBJCOPY) -O ihex -R .eeprom $(TARGET_EXEC).elf $(TARGET_EXEC).hex
	@chmod +x $(TARGET_EXEC).hex
	@$(OBJDUMP) -dstz $(BUILD_DIR)/$(TARGET_EXEC).elf > $(BUILD_DIR)/$(TARGET_EXEC).dump

# Build step for compiling C source files
$(BUILD_DIR)/%.c.o: %.c
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_C) $(CPPFLAGS) $(CFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump

# Build step for compiling C++ source files
$(BUILD_DIR)/%.cpp.o: %.cpp
	@mkdir -p $(dir $@)
	@echo [Building $<]
	@$(COMPILER_CPP) $(CPPFLAGS) $(CXXFLAGS) -c $< -o $@
	@$(OBJDUMP) -dstz $@ > $@.dump

# Clean the build directory and remove generated executables
clean:
	rm -rf $(BUILD_DIR)
	rm -rf $(TARGET_EXEC).elf $(TARGET_EXEC).hex $(TARGET_EXEC).map $(TARGET_EXEC).dump

# Clean only the source object files
clean_src:
	rm -rf $(BUILD_DIR)/src

# Clean only the library object files
clean_libs:
	rm -rf $(BUILD_DIR)/libraries

# Clean only the Teensy object files
clean_teensy4:
	rm -rf $(BUILD_DIR)/teensy4

# Include the dependency files to manage header file dependencies
-include $(TEENSY_DEPS)
-include $(LIBRARY_DEPS)
-include $(SRC_DEPS)

# Upload the firmware to the Teensy device
upload: build
	@echo [Uploading] - If this fails, press the button on the teensy and re-run 'make upload'
	@tycmd upload $(TARGET_EXEC).hex

# Install required tools for building and uploading firmware
install:
	@bash $(TOOLS_DIR)/install_tytools.sh
	@bash $(TOOLS_DIR)/install_compiler.sh

# Resets teensy and switches it into boot-loader mode
kill:
	@echo [Attempting to Kill Teensy]
	@tycmd reset -b

# Restarts teensy
restart:
	@echo [Attempting to Restart Firmware]
	@tycmd reset

help:
	@echo "Basic usage: make [target]"
	@echo "Targets:"
	@echo "  install:      installs all required dependencies (ARM toolchain and tycmd)"
	@echo "  build:        compiles the source code and links with libraries"
	@echo "  upload:       builds the source and uploads it to the Teensy"
	@echo "  clean:        removes all build artifacts"
	@echo "  kill:         stops any running firmware"
	@echo "  restart:      restarts any running firmware"
	@echo "  cdb:          generates compile_commands.json for IDE support"

# --- Compile DB generation with Bear -----------------------------------------

# Directory to host wrapper symlinks
BEAR_WRAPDIR ?= .bearwrap

# Try to find Bear's wrapper path on common installs
BEAR_WRAPPER ?= $(shell brew --prefix bear 2>/dev/null)/lib/bear/wrapper
ifeq ($(wildcard $(BEAR_WRAPPER)),)
  BEAR_WRAPPER := /usr/lib/bear/wrapper
endif
ifeq ($(wildcard $(BEAR_WRAPPER)),)
  BEAR_WRAPPER := /usr/lib/x86_64-linux-gnu/bear/wrapper
endif

# Generate compile_commands.json for clangd
cdb:
	@command -v bear >/dev/null || { echo "Error: bear not found in PATH. Install with: brew install bear"; exit 1; }
	@test -x "$(BEAR_WRAPPER)" || { echo "Error: bear wrapper not found at $(BEAR_WRAPPER)"; exit 1; }
	@echo "[cdb] Preparing Bear wrapper links in $(BEAR_WRAPDIR)"
	@rm -f compile_commands.json compile_commands.events.json
	@mkdir -p $(BEAR_WRAPDIR)
	@ln -sf "$(BEAR_WRAPPER)" "$(BEAR_WRAPDIR)/arm-none-eabi-g++"
	@ln -sf "$(BEAR_WRAPPER)" "$(BEAR_WRAPDIR)/arm-none-eabi-gcc"
	@echo "[cdb] Running build through Bear to capture commands"
	@PATH="$(COMPILER_TOOLS_PATH):$$PATH" \
	bear --wrapper-dir "$(BEAR_WRAPDIR)" -- \
	  $(MAKE) -B build \
	  COMPILER_CPP=arm-none-eabi-g++ \
	  COMPILER_C=arm-none-eabi-gcc
	@{ command -v jq >/dev/null && jq 'length' compile_commands.json; } >/dev/null 2>&1 || true
	@echo "[cdb] Done: compile_commands.json generated"
