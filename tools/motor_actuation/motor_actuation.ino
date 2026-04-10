// Standalone bench test for the A4988 stepper circuit (Arduino Mega).
// Open this sketch in Arduino IDE and upload separately — not part of the Teensy firmware.
#include <AccelStepper.h>

// ── Pin definitions ───────────────────────────────────────────────────────
#define STEP_PIN        2               // STEP on A4988
#define DIR_PIN         3               // DIR on A4988

// ── Stepper configuration ──────────────────────────────────────────────────
#define STEPS_PER_REV   200             // NEMA 17 = 200 steps/rev
#define MAX_SPEED       800.0           // steps/s
#define ACCELERATION    400.0           // steps/s^2

// ── Deployment parameters — tune these on the bench ───────────────────────
#define DEPLOY_STEPS    400             // steps to full extension
#define DEPLOY_TIME_MS  12000           // 12s — burnout (9.83s) + 2s margin
#define RETRACT_TIME_MS 23000           // 23s — apogee (26.42s) - 3s margin

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool deployed  = false;
bool retracted = false;

void setup() {
    Serial.begin(115200);

    // ENA is tied to 3.3V in the circuit (always enabled) so no pinMode needed
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);     // home = fully retracted

    Serial.println("Ready. Waiting for launch...");
}

void loop() {
    unsigned long t = millis();

    // ── Deploy ────────────────────────────────────────────────────────────
    if (!deployed && t >= DEPLOY_TIME_MS) {
        Serial.println("Deploying airbrakes...");
        stepper.moveTo(DEPLOY_STEPS);
        deployed = true;
    }

    // ── Retract ───────────────────────────────────────────────────────────
    if (deployed && !retracted && t >= RETRACT_TIME_MS) {
        Serial.println("Retracting airbrakes...");
        stepper.moveTo(0);
        retracted = true;
    }

    // ── Run stepper ───────────────────────────────────────────────────────
    stepper.run();

    // ── Status print every 500ms ──────────────────────────────────────────
    static unsigned long lastPrint = 0;
    if (t - lastPrint > 500) {
        Serial.print("t=");        Serial.print(t / 1000.0, 2);
        Serial.print("s  pos=");   Serial.print(stepper.currentPosition());
        Serial.print("  target="); Serial.println(stepper.targetPosition());
        lastPrint = t;
    }
}
