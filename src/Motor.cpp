#include "Motor.h"
#include <Arduino.h>

Motor::Motor(uint8_t pulsePin, uint8_t dirPin, char axis, uint16_t microStepMultiplier)
{
    this->StepsPerRev = 200;
    this->PulsePin = pulsePin;
    this->DirPin = dirPin;
    this->Axis = axis;
    this->MicrostepMultiplier = microStepMultiplier;
    this->StepsRemaining = 0;
    this->BaseMetricInSteps = 152.86625;
    this->MaxSpeed = 400;

    this->ShortDistance = 8 * this->BaseMetricInSteps;
    this->DwellSpeed = 10;
    this->RampStartsAt = 20;

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
                this->TotalStepsTaken++;

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
    this->CurrentSpeed = speed;

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
    uint8_t inHigh = this->IsDirInverted ? LOW : HIGH;
    uint8_t inLow = this->IsDirInverted ? HIGH : LOW;
    
    digitalWrite(this->DirPin, direction == 1 ? inHigh : inLow);
    this->Direction = direction;
}

uint16_t Motor::GetSpeed() {
    return this->CurrentSpeed;
}