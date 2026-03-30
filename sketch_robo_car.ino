#include <Wire.h>
#include <OPT3101.h>
#include <Servo.h>

// WHEELS 

// M2_PWM1 = rear left wheel
// M2_PWM2 = rear right wheel
// M1_PWM2 = front left wheel
// M1_PWM1 = front right wheel

#define M2_DIR1 22
#define M2_DIR2 24
#define M2_PWM1 5
#define M2_PWM2 6

#define M1_DIR1 26
#define M1_DIR2 28
#define M1_PWM1 7
#define M1_PWM2 8

#define AFSTAND_L 0
#define AFSTAND_M 1
#define AFSTAND_R 2

#define AFSTAND 150
// ENCODERS
// #define ENC_RL_A 2
// #define ENC_RR_A 3
// #define ENC_FL_A 18
// #define ENC_FR_A 19

// #define ENC_RL_B 30
// #define ENC_RR_B 32
// #define ENC_FL_B 34
// #define ENC_FR_B 36

OPT3101 sensor;
Servo servo1;

const uint8_t dataReadyPin = 18;

volatile bool dataReady = false;
bool AfstandRenewedCycle = false;
unsigned long stateStartTime = 0;

enum RobotState
{
  DRIVE_FORWARD,
  TURN_LEFT,
  TURN_RIGHT,
  BACKING_UP,
  BACKING_UP_OBS_RIGHT,
  BACKING_UP_OBS_LEFT,
};

void setDataReadyFlag()
{
  dataReady = true;
}

RobotState robotState = DRIVE_FORWARD;

const int DETECTION_DISTANCE_MM = 300;
const uint16_t MIN_VALID_AMPLITUDE = 150;

const uint8_t CH_LEFT   = 0;
const uint8_t CH_CENTER = 1;
const uint8_t CH_RIGHT  = 2;

uint16_t amplitudes[3] = {0, 0, 0};
int16_t distances[3] = {0, 0, 0};
bool validReading[3] = {false, false, false};


bool obstacleLeft = false;
bool obstacleCenter = false;
bool obstacleRight = false;

int speedForward = 70;
int speedTurn = 100;
int speedBackward = 100;

const unsigned long FORWARD_STEP_MS = 30;
const unsigned long BACKUP_STEP_MS = 100;
const unsigned long TURN_STEP_MS = 30;

const unsigned long BACKUP_TOTAL_MS = 180;
const unsigned long TURN_SMALL_TOTAL_MS = 260;
const unsigned long TURN_LARGE_TOTAL_MS = 500;

int blockedCount = 0;

void stopMotors()
{
  analogWrite(M1_PWM1, 0);
  analogWrite(M1_PWM2, 0);
  analogWrite(M2_PWM1, 0);
  analogWrite(M2_PWM2, 0);
}

void changeRobotState(RobotState state)
{
  robotState = state;
  stateStartTime = millis();
}

void forward()
{
  digitalWrite(M2_DIR1, LOW);
  digitalWrite(M2_DIR2, LOW);
  digitalWrite(M1_DIR1, HIGH);
  digitalWrite(M1_DIR2, HIGH);

  analogWrite(M2_PWM1, speedForward);
  analogWrite(M2_PWM2, speedForward);
  analogWrite(M1_PWM1, speedForward);
  analogWrite(M1_PWM2, speedForward);
}

void backward()
{
  digitalWrite(M2_DIR1, HIGH);
  digitalWrite(M2_DIR2, HIGH);
  digitalWrite(M1_DIR1, LOW);
  digitalWrite(M1_DIR2, LOW);

  analogWrite(M2_PWM1, speedBackward);
  analogWrite(M2_PWM2, speedBackward);
  analogWrite(M1_PWM1, speedBackward);
  analogWrite(M1_PWM2, speedBackward);
}

void turnLeft()
{
  // left side backward, right side forward
  digitalWrite(M2_DIR1, HIGH);   // rear left
  digitalWrite(M1_DIR1, LOW);    // front left

  digitalWrite(M2_DIR2, LOW);    // rear right
  digitalWrite(M1_DIR2, HIGH);   // front right

  analogWrite(M2_PWM1, speedTurn);
  analogWrite(M2_PWM2, speedTurn);
  analogWrite(M1_PWM1, speedTurn);
  analogWrite(M1_PWM2, speedTurn);
}

void turnRight()
{
  // left side forward, right side backward
  digitalWrite(M2_DIR1, LOW);    // rear left
  digitalWrite(M1_DIR1, HIGH);   // front left

  digitalWrite(M2_DIR2, HIGH);   // rear right
  digitalWrite(M1_DIR2, LOW);    // front right

  analogWrite(M2_PWM1, speedTurn);
  analogWrite(M2_PWM2, speedTurn);
  analogWrite(M1_PWM1, speedTurn);
  analogWrite(M1_PWM2, speedTurn);
}



void setup()
{
  pinMode(M2_DIR1, OUTPUT);
  pinMode(M2_DIR2, OUTPUT);
  pinMode(M2_PWM1, OUTPUT);
  pinMode(M2_PWM2, OUTPUT);

  pinMode(M1_DIR1, OUTPUT);
  pinMode(M1_DIR2, OUTPUT);
  pinMode(M1_PWM1, OUTPUT);
  pinMode(M1_PWM2, OUTPUT);

  servo1.attach(9);

  // pinMode(ENC_RL_A, INPUT_PULLUP);
  // pinMode(ENC_RR_A, INPUT_PULLUP);
  // pinMode(ENC_FL_A, INPUT_PULLUP);
  // pinMode(ENC_FR_A, INPUT_PULLUP);

  // pinMode(ENC_RL_B, INPUT_PULLUP);
  // pinMode(ENC_RR_B, INPUT_PULLUP);
  // pinMode(ENC_FL_B, INPUT_PULLUP);
  // pinMode(ENC_FR_B, INPUT_PULLUP);

  Serial.begin(9600);
  Wire.begin();

  sensor.init();
  if (sensor.getLastError())
  {
    Serial.print("Failed to initialize OPT3101: error ");
    Serial.println(sensor.getLastError());
    while (1) {}
  }

  sensor.setContinuousMode();
  sensor.enableDataReadyOutput(1);
  sensor.setFrameTiming(32);
  sensor.setChannel(OPT3101ChannelAutoSwitch);
  sensor.setBrightness(OPT3101Brightness::Adaptive);

  attachInterrupt(digitalPinToInterrupt(dataReadyPin), setDataReadyFlag, RISING);
  sensor.enableTimingGenerator();
  
}

void getSensorData()
{

  if (!dataReady)
    return;
  dataReady = false;
  // Serial.println("Data ready");
  sensor.readOutputRegs();

  // Serial.print("sensor.amplitude: ");
  // Serial.println(sensor.amplitude);

  // Serial.print("sensor.distanceMillimeters: ");
  // Serial.println(sensor.distanceMillimeters);

  amplitudes[sensor.channelUsed] = sensor.amplitude;
  distances[sensor.channelUsed] = sensor.distanceMillimeters;
  if (sensor.channelUsed == 2)
  {
    AfstandRenewedCycle = true;
  }
  // if (sensor.channelUsed == 1)
  // {
  //   for (uint8_t i = 0; i < 3; i++)
  //   {
  //     Serial.print(amplitudes[i]);
  //     Serial.print(',');
  //     Serial.print(distances[i]);
  //     Serial.print(", ");
  //   }
  //   Serial.println();
  // }

}

void updateNavigation()
{
  unsigned long elapsed = millis() - stateStartTime;

  switch (robotState)
  {
    case BACKING_UP:
      if (elapsed < BACKUP_TOTAL_MS)
        return;
      stopMotors();
      turnLeft();
      changeRobotState(TURN_LEFT);
      return;
    case TURN_LEFT:
      if (elapsed < TURN_LARGE_TOTAL_MS) return;
      stopMotors();
      changeRobotState(DRIVE_FORWARD);
      return;
    case TURN_RIGHT:
      if (elapsed < TURN_LARGE_TOTAL_MS) return;
      stopMotors();
      changeRobotState(DRIVE_FORWARD);
      return;
    case DRIVE_FORWARD:
      break;
    case BACKING_UP_OBS_RIGHT:
      // Serial.print("backing up state, elapsed: ");
      // Serial.print(elapsed);
      // Serial.print(" dist[2]: ");
      // Serial.println(distances[2]);
      if (elapsed < 200)
        return;
      stopMotors();
      turnLeft();
      changeRobotState(TURN_LEFT);
      return;
    case BACKING_UP_OBS_LEFT:
      // Serial.print("backing up state, elapsed: ");
      // Serial.print(elapsed);
      // Serial.print(" dist[2]: ");
      // Serial.println(distances[2]);
      if (elapsed < 200)
        return;
      stopMotors();
      turnRight();
      changeRobotState(TURN_RIGHT);
      return;
  }
  if (!obstacleCenter)
  {
    forward();
    changeRobotState(DRIVE_FORWARD);
  }
  else if (!obstacleLeft && obstacleRight)
  {
    stopMotors();
    if (distances[2] < 150)
    {
      // Serial.print("right obstacle too close: ");
      // Serial.print("R:"); Serial.print(distances[2]); 
      // Serial.print(" chUsed:"); Serial.print(sensor.channelUsed);
      // Serial.print(" obsR:"); Serial.println(obstacleRight);
      backward();
      changeRobotState(BACKING_UP_OBS_RIGHT);
      return;
    }
    turnLeft();
    changeRobotState(TURN_LEFT);

  }
  else if (!obstacleRight && obstacleLeft)
  {
    if (distances[0] < 150)
    {
      backward();
      changeRobotState(BACKING_UP_OBS_LEFT);
      return;
    }
    stopMotors();
    turnRight();
    changeRobotState(TURN_RIGHT);

  }
  else if (!obstacleRight && !obstacleLeft)
  {
    stopMotors();
    if (distances[0] >= distances[2])
    {
      turnLeft();
      changeRobotState(TURN_LEFT);
    }
    else
    {
      turnRight();
      changeRobotState(TURN_RIGHT);
    }
  }
  else
  {
    stopMotors();
    backward();
    changeRobotState(BACKING_UP);

  }
}

void updateAfstandOutputValues()
{
  obstacleLeft = distances[0] > 0 && distances[0] < AFSTAND + 60;
  obstacleCenter = distances[1] > 0 && distances[1] < AFSTAND;
  obstacleRight = distances[2] > 0 && distances[2] < AFSTAND + 60;
}

void loop()
{
  getSensorData();

  if (AfstandRenewedCycle)
  {
    AfstandRenewedCycle = false;
    updateAfstandOutputValues();
    // Serial.print("L:"); Serial.print(distances[0]); Serial.print(" A:"); Serial.print(amplitudes[0]);
    // Serial.print(" obsL:"); Serial.println(obstacleLeft);
    // Serial.print("C:"); Serial.print(distances[1]); Serial.print(" obsC:"); Serial.println(obstacleCenter);
    // Serial.print("R:"); Serial.print(distances[2]); Serial.print(" A:"); Serial.print(amplitudes[2]);
    // Serial.print(" obsR:"); Serial.println(obstacleRight);
  }
  updateNavigation();

}