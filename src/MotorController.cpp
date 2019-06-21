#include <Arduino.h>
#include "MotorController.h"

MotorController::MotorController()
{
}

void MotorController::Initialize()
{
}

void MotorController::Offset()
{
    Serial.println("Offset: Basliyor");

    Motor *vargel = this->Motors[1];
    uint8_t state = digitalRead(vargel->MinSwitchPin);

    vargel->SetDirection(Backwards);
    unsigned long lastSwitchReadAt = millis();

    while (state == HIGH)
    {
        vargel->Step();

        if (millis() - lastSwitchReadAt > 30)
        {
            lastSwitchReadAt = millis();
            state = digitalRead(vargel->MinSwitchPin);
        }
    }

    Serial.println(F("Offset: Bitti"));
}

void MotorController::Move(char axis, long steps)
{
    Motor *target = this->Find(axis);

    if (target != NULL)
    {
        Serial.println(target->Axis);
        Serial.println(steps);

        if (steps < 0)
            target->SetDirection(Backwards);
        else
            target->SetDirection(Forwards);

        uint8_t covered = 0;
        uint8_t delta = abs(steps);

        while (covered < delta)
        {
            target->Step();
            covered++;
        }
    }
}

Motor *MotorController::Find(char axis)
{
    for (uint8_t i = 0; i < 2; i++)
    {
        if (this->Motors[i]->Axis == axis)
            return this->Motors[i];
    }

    return NULL;
}