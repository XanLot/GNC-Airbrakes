#ifndef STEPPER_HPP
#define STEPPER_HPP

#include <AccelStepper.h>

class Stepper {
public:
    Stepper();

    void init();   // call once in setup()
    void run();    // call every loop() — generates step pulses

    void deployTo(int steps);
    void retractToHome();

    int  currentPosition();
    int  targetPosition();
    bool isRunning();

private:
    AccelStepper stepper_;

    static constexpr float MAX_SPEED    = 800.0f;   // steps/s
    static constexpr float ACCELERATION = 400.0f;   // steps/s^2
};

#endif // STEPPER_HPP
