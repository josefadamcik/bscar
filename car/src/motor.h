#include <Arduino.h>
#include <../../shared/shared_structs.h>

struct MotorConfig {
  int pwmPin;
  int forwardPin;
  int backwardPin;
  MotorConfig(int pwm, int fw, int bw) : pwmPin(pwm), forwardPin(fw), backwardPin(bw) {}
};

void motor_init(MotorConfig &motor) {
  pinMode(motor.pwmPin, OUTPUT);
  pinMode(motor.forwardPin, OUTPUT);
  pinMode(motor.backwardPin, OUTPUT);
}

MotorState motor_go(MotorConfig &motor, int dir, int speed) {
  digitalWrite(motor.forwardPin, dir == DIR_FW ? HIGH : LOW);
  digitalWrite(motor.backwardPin, dir == DIR_FW ? LOW : HIGH);
  analogWrite(motor.pwmPin, speed);
  return MotorState(speed, speed == 0 ? DIR_NONE : dir);
}

MotorState motor_stop(MotorConfig &motor) {
  digitalWrite(motor.forwardPin, LOW);
  digitalWrite(motor.backwardPin, LOW);
  analogWrite(motor.pwmPin, 0);
  return MotorState();
}


