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
    void Offset();
};

#endif