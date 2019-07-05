#ifndef MOTORCONTROLLER_H
#define MOTORCONTROLLER_H

#include "Motor.h"

class MotorController
{
private:
protected:
public:
    Motor *Motors[2];
    MotorController();
    void Initialize();
    void Offset(const char* position);
    void BlockMove(char axis, long delta, uint16_t speed);
    void Move(char axis, long delta, uint16_t speed);
    Motor* Find(char axis);
    bool IsCompleted();
    void Sync();
    void Halt();
    void LinearMove(char firstAxis, char secondAxis, long firstDelta, long secondDelta, uint16_t speed);
    void CalculateRamp(unsigned long delta, unsigned long index, Motor *motor);
    void FinishLinearMove();
    void RampDown();

    long KarkasBeginsAt;
    long KarkasEndsAt;
    float BaseMetricInSteps;
    bool IsMovingLinear;
    bool ShouldRampUp;
    bool ShouldRampDown;

    unsigned long LinearIndex, LinearOvershoot, DeltaX, DeltaY;
};

#endif