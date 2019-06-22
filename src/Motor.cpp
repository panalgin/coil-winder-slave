#include "Motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t pulsePin, uint8_t dirPin, char axis, uint16_t microStepMultiplier)
{
    this->PulsePin = pulsePin;
    this->DirPin = dirPin;
    this->Axis = axis;
    this->MicrostepMultiplier = microStepMultiplier;
    this->StepsRemaining = 0;

    pinMode(this->PulsePin, OUTPUT);
    pinMode(this->DirPin, OUTPUT);

    this->LastSteppedAt = 0;
    this->StepInterval = 100;

    this->SetSpeed(200);
}

void Motor::Step()
{
    if (this->StepsRemaining > 0) {
        while (true)
        {
            if (micros() - this->LastSteppedAt > this->StepInterval)
            {
                this->LastSteppedAt = micros();

                digitalWrite(this->PulsePin, HIGH);
                delayMicroseconds(1);
                digitalWrite(this->PulsePin, LOW);
                delayMicroseconds(1);

                this->StepsRemaining--;

                if (this->Direction == -1)
                    this->CurrentPosition--;
                else
                    this->CurrentPosition++;
                    
                break;
            }
        }
    }
}

void Motor::SetSpeed(uint16_t speed)
{
    unsigned long divider = (unsigned long)speed * (unsigned long)200 * (unsigned long)this->MicrostepMultiplier;
    this->StepInterval = (60000000 / divider) - 2;
    this->MaxSpeed = speed;

    /*/Serial.print("Interval: ");
    Serial.println(this->StepInterval);

    Serial.print("Divider: ");
    Serial.println(divider);*/
}

void Motor::SetLimitSwitches(uint8_t minSwitchPin, uint8_t maxSwitchPin)
{
    pinMode(minSwitchPin, INPUT_PULLUP);
    pinMode(maxSwitchPin, INPUT_PULLUP);

    this->MinSwitchPin = minSwitchPin;
    this->MaxSwitchPin = maxSwitchPin;
}

void Motor::SetDirection(int8_t direction)
{
    digitalWrite(this->DirPin, direction == 1 ? HIGH : LOW);
    this->Direction = direction;
}