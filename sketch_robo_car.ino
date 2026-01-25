#include <AFMotor.h>
//#include <Servo.h>


#define BASESPEED 200

class Motor {
private:
  AF_DCMotor _motor;
  int        _baseDir;

public:
  Motor(int num): _motor(num) {
    setSpeed(BASESPEED);
  }
  Motor(int num, int motorDir): _motor(num), _baseDir(motorDir) {
    setSpeed(BASESPEED);
  }

  void setSpeed(int speed) {
    _motor.setSpeed(speed);
  }

  void moveForward() {
    if (_baseDir == BACKWARD) {
      _motor.run(BACKWARD);
    } else {
      _motor.run(FORWARD);
    }
  }

  void moveBackward() {
    if (_baseDir == BACKWARD) {
      _motor.run(FORWARD);
    } else {
      _motor.run(BACKWARD);
    }
  }

  void move(int direction) {
    if (direction == FORWARD) {
      moveForward();
    } else {
      moveBackward();
    }
  }

  void stop() {
    _motor.run(RELEASE);
  }
};


class Car {
private:
  Motor _motors[4];

public:
  Car(int m1Dir, int m2Dir,int m3Dir,int m4Dir): 
    _motors({ Motor(1, m1Dir), Motor(2, m2Dir), Motor(3, m3Dir), Motor(4, m4Dir)}) 
  {
  }

  void moveRight() {
    _motors[0].moveForward();
    _motors[1].moveForward();
    _motors[2].moveBackward();
    _motors[3].moveBackward();
  }

  void moveLeft() {
    _motors[0].moveBackward();
    _motors[1].moveBackward();
    _motors[2].moveForward();
    _motors[3].moveForward();
  }

  void moveForward() {
    for (auto motor : _motors) {
      motor.moveForward();
    }
  }

  void moveBackward() {
    for (auto motor : _motors) {
      motor.moveBackward();
    }
  }

  void stop() {
    for (auto motor : _motors) {
      motor.stop();
    }

  }
};


Car car(BACKWARD, FORWARD, FORWARD, BACKWARD);

void setup() {
}

void loop() {
  car.moveForward();
  delay(3000);
  car.stop();
  delay(3000);

  car.moveBackward();
  delay(3000);
  car.stop();
  delay(3000);

  car.moveLeft();
  delay(3000);
  car.stop();
  delay(3000);

  car.moveRight();
  delay(3000);
  car.stop();
  delay(10000);

}
