#include <Arduino.h>
#include "MotorController.h"

MotorController::MotorController()
{
  this->BaseMetricInSteps = 152.86625;
}

void MotorController::Initialize()
{

}

void MotorController::Offset(const char* position)
{
  Motor *y = this->Find('Y');

  if (strcmp(position, "First") == 0)
  {
    y->CurrentPosition = 0;
    this->KarkasBeginsAt = y->CurrentPosition;

    Serial.print("Baslangic: ");
    Serial.println(this->KarkasBeginsAt);
  }
  else if (strcmp(position, "Second") == 0)
  {
    this->KarkasEndsAt = y->CurrentPosition;

    Serial.print("Bitis: ");
    Serial.println(this->KarkasEndsAt);
  }
}

void MotorController::Move(char axis, long steps, uint16_t speed)
{
  Motor *target = this->Find(axis);

  if (target != NULL)
  {
    Serial.println(target->Axis);
    Serial.println(steps);

    target->SetSpeed(speed);

    if (steps < 0)
      target->SetDirection(Backwards);
    else
      target->SetDirection(Forwards);

    long delta = abs(steps);
    target->StepsRemaining = delta;
  }
}

bool MotorController::IsCompleted()
{
  Motor *x = this->Motors[0];
  Motor *y = this->Motors[1];

  return x->StepsRemaining == 0 && y->StepsRemaining == 0;
}

void MotorController::Sync()
{
  for (uint8_t i = 0; i < 2; i++)
  {
    Motor *target = this->Motors[i];

    if (target->StepsRemaining > 0)
      target->Step();
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

void MotorController::Halt()
{
  Motor *x = this->Motors[0];
  Motor *y = this->Motors[1];

  x->StepsRemaining = 0;
  y->StepsRemaining = 0;
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
