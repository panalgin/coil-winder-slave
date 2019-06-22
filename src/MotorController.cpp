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

void MotorController:: Move(char axis, long steps)
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

        long covered = 0;
        long delta = abs(steps);

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

/*/void MotorController::LinearMove(long steps1, long steps2, Motor *first, Motor *second) {
  long pos1 = first->GetCurrentPosition() + steps1;
  long pos2 = second->GetCurrentPosition() + steps2;

  this->LinearMoveTo(pos1, pos2, first, second);
}

void MotorController::LinearMoveTo(long pos1, long pos2, Motor *first, Motor *second) {  
  first->PrepareTo(pos1);
  second->PrepareTo(pos2);

  long m_DeltaX = first->StepsRemaining;
  long m_DeltaY = second->StepsRemaining;

  long m_Index;
  long m_Over = 0;

  if (m_DeltaX > m_DeltaY) {    
    for(m_Index = 0; m_Index < m_DeltaX; ++m_Index) { 
      if (first->UseRamping)     
        this->CalculateRamp(m_DeltaX, m_Index, first);

      first->Step();
      m_Over += m_DeltaY;

      if (m_Over >= m_DeltaX) {
        m_Over -= m_DeltaX;
        second->Step();
      }
    }

    first->SetSpeed(first->MaxSpeed);
  }
  else {
    for(m_Index = 0; m_Index < m_DeltaY; ++m_Index) { 
      if (second->UseRamping)     
        this->CalculateRamp(m_DeltaY, m_Index, second);

      second->Step();
      m_Over += m_DeltaX;

      if (m_Over >= m_DeltaY) {
        m_Over -= m_DeltaY;
        first->Step();
      }
    }

    second->SetSpeed(second->MaxSpeed);
  }
}*/
