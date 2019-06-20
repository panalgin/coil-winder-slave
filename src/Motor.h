#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define Backwards -1
#define Forwards 1
class Motor
{
private:
    uint8_t PulsePin;
    uint8_t DirPin;
    uint16_t MicrostepMultiplier;

protected:
    uint16_t CurrentSpeed;
    uint16_t MaxSpeed;

public:
    unsigned long LastSteppedAt;
    unsigned long StepInterval;
    char Axis;

    uint8_t MinSwitchPin;
    uint8_t MaxSwitchPin;

    int8_t Direction;

    Motor(uint8_t pulsePin, uint8_t dirPin, char axis, uint16_t microStepMultiplier);
    void SetLimitSwitches(uint8_t minSwitchPin, uint8_t maxSwitchPin);
    void Step();
    void SetSpeed(uint16_t speed);
    void SetDirection(int8_t direction);
};

#endif