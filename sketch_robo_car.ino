#include <Wire.h>
#include <OPT3101.h>

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

#define AFSTAND 200
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

const uint8_t dataReadyPin = 18;

volatile bool dataReady = false;
bool AfstandRenewedCycle = false;


void setDataReadyFlag()
{
  dataReady = true;
}


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

int speedForward = 55;
int speedTurn = 55;
int speedBackward = 50;

const unsigned long FORWARD_STEP_MS = 30;
const unsigned long BACKUP_STEP_MS = 35;
const unsigned long TURN_STEP_MS = 30;

const unsigned long BACKUP_TOTAL_MS = 180;
const unsigned long TURN_SMALL_TOTAL_MS = 260;
const unsigned long TURN_LARGE_TOTAL_MS = 500;

int blockedCount = 0;

enum RobotState
{
  DRIVE_FORWARD,
  BACKING_UP,
  TURNING_LEFT,
  TURNING_RIGHT
};

RobotState robotState = DRIVE_FORWARD;
unsigned long stateStartTime = 0;
unsigned long lastMotionUpdate = 0;

void stopMotors()
{
  analogWrite(M1_PWM1, 0);
  analogWrite(M1_PWM2, 0);
  analogWrite(M2_PWM1, 0);
  analogWrite(M2_PWM2, 0);
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

  if (dataReady)
  {
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
}

void updateAfstandOutputValues()
{
  
  // bool obstacleLeft = false;
  // bool obstacleCenter = false;
  // bool obstacleRight = false;
  // && amplitudes[0] > 
  obstacleLeft = distances[0] > 0 && distances[0] < AFSTAND;
  obstacleCenter = distances[1] > 0 && distances[1] < AFSTAND;
  obstacleRight = distances[2] > 0 && distances[2] < AFSTAND;
}

void loop()
{
  getSensorData();

  if (AfstandRenewedCycle)
  {
    AfstandRenewedCycle = false;
    updateAfstandOutputValues();
    if (obstacleCenter)
    {
      if (obstacleRight && !obstacleLeft)
      {
        stopMotors();
        delay(400);
        backward();
        delay(400);
        turnLeft();
        delay(400);
        stopMotors();
      }
      else if (obstacleLeft && !obstacleRight)
      {
        stopMotors();
        delay(400);
        backward();
        delay(400);
        turnRight();
        delay(400);
        stopMotors();
      }
      else
      {
        stopMotors();
        delay(400);
        backward();
        delay(900);
        turnLeft();
        delay(4000);
        stopMotors();
      }
    }
    else
    {
      forward();
    }
  }

  // forward();
  // delay(4000);
  // stopMotors();
  // delay(1000);

}