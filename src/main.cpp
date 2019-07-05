#include <Arduino.h>
#include <SoftwareSerial.h>
#include "Queue.h"
#include "MotorController.h"

typedef struct
{
  char Axis[2];
  long Delta[2];
  uint16_t Speed;
} Parameter;

typedef struct
{
  uint8_t No;
  Parameter Params;
} Gcode;

typedef struct
{
  uint16_t SpirIndex;
  uint16_t SpirTotal;
  uint16_t WorkingCycle;
  bool IsPaused;
  float WireDiameter;
  float TurnsPerLayer;
  uint16_t Speed;
  bool IsAtEnd;
} Job;

Job CJob = Job();
Job *CurrentJob;

Motor mainMotor(8, 5, 'X', 8);
Motor vargelMotor(9, 6, 'Y', 16);

MotorController controller;
Queue<Gcode> Codes(16);

SoftwareSerial com(12, 11);

void parseMessage(String *message);
void loopSteps();
void parseWork(String *message);
String SplitValues(String *data, char seperator, uint8_t index);
void pauseWork();

void setup()
{
  delay(100);
  com.begin(57600);

  while (!com)
  {
    ;
  }

  Serial.begin(115200);

  controller.Initialize();

  vargelMotor.SetLimitSwitches(A1, A3);
  vargelMotor.IsDirInverted = true;
  vargelMotor.SetSpeed(400);

  controller.Motors[0] = &mainMotor;
  controller.Motors[1] = &vargelMotor;
}

String incomingMessage = "";

void loop()
{
  if (com.available())
  {
    char c = com.read();

    Serial.print(c);

    if (c != '\n')
      incomingMessage += c;
    else
      parseMessage(&incomingMessage);
  }

  loopSteps();
}

bool supposedToRun = false;
long lastSteps = 0;
uint16_t lastSpirIndex = 0;

void loopSteps()
{
  if (Codes.count() > 0 && controller.IsCompleted())
  {
    Gcode code = Codes.pop();
    Parameter params = code.Params;

    if (code.No == 1)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed;

      controller.Move(axis, (long)(delta > 0 ? INT32_MAX : INT32_MIN), speed);
    }
    else if (code.No == 0)
    {
      char axis = params.Axis[0];
      long delta = params.Delta[0];
      uint16_t speed = params.Speed;

      controller.Move(axis, delta, speed);
    }
    else if (code.No == 2)
    {
      char firstAxis = params.Axis[0];
      char secondAxis = params.Axis[1];

      long firstDelta = params.Delta[0];
      long secondDelta = params.Delta[1];

      uint16_t maxSpeed = params.Speed;

      controller.LinearMove(firstAxis, secondAxis, firstDelta, secondDelta, maxSpeed);
    }
  }

  if (!controller.IsCompleted())
  {
    if (CurrentJob != NULL)
    {
      if (!CurrentJob->IsPaused || controller.ShouldRampDown)
      {
        controller.Sync();
      }
    }
    else
      controller.Sync();
  }
  else
  {
    if (CurrentJob != NULL && CurrentJob->IsAtEnd == false)
    {
      CurrentJob->IsAtEnd = true;

      com.print("CycleFinished: ");
      com.println(CurrentJob->WorkingCycle);

      Serial.print("CycleFinished: ");
      Serial.println(CurrentJob->WorkingCycle);
    }
  }
}

void parseMessage(String *message)
{
  message->replace("\r", "");

  if (message->startsWith("Pause"))
    pauseWork();

  else if (message->startsWith("Offset-Main"))
  {
    Parameter p = {{'X'}, {1}, 10};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("OMD"))
  {
    Codes.clear();
    controller.Halt();
  }
  else if (message->startsWith("Offset-First"))
  {
    controller.Offset("First");

    float firstPos = (float)(controller.KarkasBeginsAt / controller.BaseMetricInSteps);
    com.print("OFD: ");
    com.println(firstPos);
  }
  else if (message->startsWith("Offset-Second"))
  {
    message->replace("Offset-Second ", "");

    float karkasWidth = message->toFloat();

    Motor *y = controller.Find('Y');
    long delta = (long)(karkasWidth * y->BaseMetricInSteps);

    controller.BlockMove('Y', delta, 300);

    controller.Offset("Second");

    float secondPos = (float)(controller.KarkasEndsAt / controller.BaseMetricInSteps);
    com.print("OSD: ");
    com.println(secondPos);

    //controller.Move('Y', delta, 300);

    delay(500);

    delta = y->CurrentPosition * -1;
    controller.BlockMove('Y', delta, 300);
  }

  else if (message->startsWith("Left"))
  {
    Parameter p = {{'Y'}, {-10}, 10};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Right"))
  {
    Parameter p = {{'Y'}, {10}, 10};
    Gcode code = {1, p};

    Codes.push(code);
  }
  else if (message->startsWith("Stop"))
  {
    Codes.clear();
    controller.Halt();
  }
  else if (message->startsWith("Work: "))
  {
    parseWork(message);
  }

  *message = "";
}

void pauseWork()
{
  if (CurrentJob != NULL)
  {
    controller.ShouldRampDown = true;
    CurrentJob->IsPaused = true;
  }
}

void parseWork(String *message)
{
  message->replace("Work: ", "");

  uint16_t currentCycle = SplitValues(message, '|', 0).toInt();
  float wireDiameter = SplitValues(message, '|', 1).toFloat();
  uint16_t speed = (uint16_t)SplitValues(message, '|', 2).toInt();

  float totalGap = (controller.KarkasEndsAt - controller.KarkasBeginsAt) / controller.BaseMetricInSteps;
  float turnsPerLayer = totalGap / wireDiameter;

  if (CurrentJob == NULL)
  {
    CurrentJob = &CJob;
    CurrentJob->IsPaused = false;
    CurrentJob->Speed = speed;
    CurrentJob->IsAtEnd = false;
    CurrentJob->SpirIndex = 0;
    CurrentJob->TurnsPerLayer = turnsPerLayer;
    CurrentJob->WireDiameter = wireDiameter;
    CurrentJob->WorkingCycle = currentCycle;

    long firstDelta = (unsigned long)(mainMotor.StepsPerRev * mainMotor.MicrostepMultiplier) * turnsPerLayer;
    long secondDelta = (totalGap * vargelMotor.BaseMetricInSteps) * (currentCycle % 2 == 0 ? 1.0f : -1.0f);

    Parameter params = {{'X', 'Y'}, {firstDelta, secondDelta}, speed};
    Gcode code = {2, params};

    Codes.push(code);
  }
  else
  {
    CurrentJob->IsPaused = false;
    CurrentJob->Speed = speed;

    if (CurrentJob->WorkingCycle != currentCycle)
    {
      long firstDelta = (unsigned long)(mainMotor.StepsPerRev * mainMotor.MicrostepMultiplier) * turnsPerLayer;
      long secondDelta = (totalGap * vargelMotor.BaseMetricInSteps) * (currentCycle % 2 == 0 ? 1.0f : -1.0f);

      Parameter params = {{'X', 'Y'}, {firstDelta, secondDelta}, speed};
      Gcode code = {2, params};

      Codes.push(code);

      Serial.println("New path aligned.");
    }

    CurrentJob->IsAtEnd = false;
    CurrentJob->WorkingCycle = currentCycle;

    vargelMotor.SetSpeed(speed);
    mainMotor.SetSpeed(mainMotor.DwellSpeed);
    mainMotor.MaxSpeed = speed;
    controller.ShouldRampUp = true;

    //Add next reverse or forward cycle delta
  }
}

String SplitValues(String *data, char seperator, uint8_t index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data->length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++)
  {
    if (data->charAt(i) == seperator || i == maxIndex)
    {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? data->substring(strIndex[0], strIndex[1]) : "";
};