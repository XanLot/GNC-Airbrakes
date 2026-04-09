#include <AccelStepper.h>

// ── Pin definitions ────────────────────────────────────────────────────────
#define STEP_PIN        3
#define DIR_PIN         2
#define ENABLE_PIN      5

// ── Stepper configuration ──────────────────────────────────────────────────
#define STEPS_PER_REV   200             // tune to your motor
#define MAX_SPEED       400.0           // steps/s
#define ACCELERATION    400.0           // steps/s^2

// ── Deployment parameters ──────────────────────────────────────────────────
#define DEPLOY_STEPS    400             // steps = full extension, tune to your geometry
#define DEPLOY_TIME_MS  12000           // ms after launch to deploy (12s = ~burnout + 2s)
#define RETRACT_TIME_MS 30000           // ms after launch to retract (before apogee)

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool deployed  = false;
bool retracted = false;

void setup() {
    Serial.begin(115200);

    pinMode(ENABLE_PIN, OUTPUT);
    digitalWrite(ENABLE_PIN, LOW);     // enable motor (active low)

    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);
    stepper.setCurrentPosition(0);     // home position = fully retracted

    Serial.println("System ready. Waiting for launch...");
}

void loop() {
    unsigned long t = millis();

    // ── Deploy at preset time ─────────────────────────────────────────────
    if (!deployed && t >= DEPLOY_TIME_MS) {
        Serial.println("Deploying airbrakes...");
        stepper.moveTo(DEPLOY_STEPS);
        deployed = true;
    }

    // ── Retract before apogee ─────────────────────────────────────────────
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
        Serial.print("t=");    Serial.print(t / 1000.0, 2);
        Serial.print("s  pos="); Serial.print(stepper.currentPosition());
        Serial.print("  target="); Serial.println(stepper.targetPosition());
        lastPrint = t;
    }
}