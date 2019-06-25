#include <Arduino.h>
#include "MotorController.h"

MotorController::MotorController()
{
  this->BaseMetricInSteps = 152.86625;
}

void MotorController::Initialize()
{
}

void MotorController::Offset(const char *position)
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
  if (!this->IsMovingLinear)
  {
    for (uint8_t i = 0; i < 2; i++)
    {
      Motor *target = this->Motors[i];

      if (target->StepsRemaining > 0)
        target->Step();
    }
  }
  else
  {
    Motor *x = this->Motors[0];
    Motor *y = this->Motors[1];

    if (this->ShouldRampDown && (x->GetSpeed() <= x->DwellSpeed || y->GetSpeed() <= y->DwellSpeed))
      this->ShouldRampDown = false;

    if (this->ShouldRampUp && (x->GetSpeed() >= x->MaxSpeed))
      this->ShouldRampUp = false;

    if (this->DeltaX > this->DeltaY)
    {
      if (this->LinearIndex < this->DeltaX)
      {
        ++this->LinearIndex;

        this->CalculateRamp(this->DeltaX, this->LinearIndex, x);

        x->Step();
        this->LinearOvershoot += this->DeltaY;

        if (this->LinearOvershoot >= this->DeltaX)
        {
          this->LinearOvershoot -= this->DeltaX;
          y->Step();
        }
      }
      else
        this->FinishLinearMove();
    }
    else
    {
      if (this->LinearIndex < this->DeltaY)
      {
        ++this->LinearIndex;

        this->CalculateRamp(this->DeltaY, this->LinearIndex, y);
        y->Step();

        this->LinearOvershoot += this->DeltaX;

        if (this->LinearOvershoot >= this->DeltaY)
        {
          this->LinearOvershoot -= this->DeltaY;
          x->Step();
        }
      }
      else
        this->FinishLinearMove();
    }
  }
}

void MotorController::FinishLinearMove()
{
  this->IsMovingLinear = false;
  this->LinearIndex = 0;
  Motor *x = this->Find('X');
  Motor *y = this->Find('Y');

  y->SetSpeed(y->MaxSpeed);
  x->SetSpeed(x->MaxSpeed);
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

void MotorController::RampDown()
{
}

void MotorController::LinearMove(char firstAxis, char secondAxis, long firstDelta, long secondDelta, uint16_t speed)
{
  Motor *x = this->Find(firstAxis);
  Motor *y = this->Find(secondAxis);

  if (firstDelta < 0)
    x->SetDirection(Backwards);
  else
    x->SetDirection(Forwards);

  if (secondDelta < 0)
    y->SetDirection(Backwards);
  else
    y->SetDirection(Forwards);

  unsigned long firstSteps = abs(firstDelta);
  unsigned long secondSteps = abs(secondDelta);

  x->StepsRemaining = firstSteps;
  y->StepsRemaining = secondSteps;

  x->SetSpeed(speed);
  y->SetSpeed(speed);

  x->MaxSpeed = speed;
  y->MaxSpeed = speed;

  this->DeltaX = x->StepsRemaining;
  this->DeltaY = y->StepsRemaining;
  this->LinearIndex = 0;
  this->LinearOvershoot = 0;
  this->IsMovingLinear = true;
}

void MotorController::CalculateRamp(unsigned long delta, unsigned long index, Motor *motor)
{
  float currentRpm = 1.0;
  unsigned long accelerationEndsAt = motor->ShortDistance / 2;             //(long)(m_DeltaY * 0.10);
  unsigned long decelerationStartsAt = delta - (motor->ShortDistance / 2); //(long)(m_DeltaY * 0.90);

  int maxSpeedCandidate = motor->MaxSpeed;

  if (this->ShouldRampDown && motor->GetSpeed() > motor->DwellSpeed)
  {
    if (abs(index - motor->LastSpeedChangeOnDelta) > 3)
    {
      motor->LastSpeedChangeOnDelta = index;
      maxSpeedCandidate = motor->GetSpeed() - 1;
      motor->SetSpeed(maxSpeedCandidate);
    }
  }
  else if (index >= accelerationEndsAt && this->ShouldRampUp && motor->GetSpeed() < motor->MaxSpeed) {
    if (abs(index - motor->LastSpeedChangeOnDelta) > 15) {
      motor->LastSpeedChangeOnDelta = index;
      maxSpeedCandidate = motor->GetSpeed() + 1;
      motor->SetSpeed(maxSpeedCandidate);

      //Serial.println(maxSpeedCandidate);
    }
  }

  int maxRpm = max(1, maxSpeedCandidate);
  int minRpm = max(1, motor->RampStartsAt);

  if (delta < motor->ShortDistance)
  {
    if (index <= 0)
    { // performance reasons
      motor->SetSpeed(motor->DwellSpeed);
    }
  }
  else
  {
    if (index <= accelerationEndsAt || index >= decelerationStartsAt)
    {
      if (index <= accelerationEndsAt)
      {
        currentRpm = max(1, minRpm + ((maxRpm - minRpm) * ((float)index / (float)accelerationEndsAt)));
        motor->SetSpeed(currentRpm);
      }
      else if (index >= decelerationStartsAt)
      {
        currentRpm = max(1, minRpm + ((maxRpm - minRpm) * (1 - ((float)index / (float)delta)) / (1 - ((float)decelerationStartsAt / (float)delta))));
        motor->SetSpeed(currentRpm);
      }
      else
        motor->SetSpeed(maxRpm);
    }
  }
}
