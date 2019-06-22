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
    void Move(char axis, long delta, uint16_t speed);
    Motor* Find(char axis);
    bool IsCompleted();
    void Sync();
    void Halt();


    long KarkasBeginsAt;
    long KarkasEndsAt;
    float BaseMetricInSteps;
};

#endif