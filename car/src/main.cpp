#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include <constants.h>
#include <motor.h>
#include <SPI.h>
#include <SoftwareSerial.h>

const int obstacleDist = 50;

CarState carState;

//with 5 and 6 for pwm the both sides behaves the same, with 3&5 it seemed that one side (pin 3) had lower voltage
MotorConfig leftMotor = MotorConfig(5, 7, 4);
MotorConfig rightMotor = MotorConfig(6, 8, 10);

SoftwareSerial bluetooth(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); 

Servo headservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

volatile unsigned int left_step_count = 0;
volatile unsigned int right_step_count = 0;
volatile byte left_last_state = LOW;
volatile byte right_last_state = LOW;
const float stepcount = 20.00;  // 20 Slots in disk, change if different
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different
const float wheelcircumference = wheeldiameter * 3.1416;
const int mainMovementSpeedIncrement = 10;
const int mainMovementSpeedMin = 100;
const int mainMovementSpeedMax = 255;
 

void opto_interrupt() {
  byte right_state = digitalRead(OPTO_INTERUPT_RIGHT_PIN);
  byte left_state = digitalRead(OPTO_INTERUPT_LEFT_PIN);
  if (right_state == HIGH && right_last_state == LOW) {
    right_step_count++;
  }
  if (left_state == HIGH && left_last_state == LOW) {
    left_step_count++;
  }
  
  left_last_state = left_state;
  right_last_state = right_state;
}




static inline int8_t sgn(int val) {
  if (val < 0) return -1;
  if (val==0) return 0;
  return 1;
}


// ==============================
// CAR CONTROL
// ==============================

/**
 * @param speedWithSign - positive values are forward, negative backward
 */ 
void car_move(int speedWithSign) {
  carState.leftMotor = motor_go(leftMotor, sgn(speedWithSign), abs(speedWithSign));
  carState.rightMotor = motor_go(rightMotor, sgn(speedWithSign), abs(speedWithSign));  
}

void car_forward(int speed) {
  carState.leftMotor = motor_go(leftMotor, DIR_FW, speed);
  carState.rightMotor = motor_go(rightMotor, DIR_FW, speed);
}

void car_backward(int speed) {
  carState.leftMotor = motor_go(leftMotor, DIR_BW, speed);
  carState.rightMotor = motor_go(rightMotor, DIR_BW, speed);
}

void car_stop() {
  carState.leftMotor = motor_stop(leftMotor);
  carState.rightMotor = motor_stop(rightMotor);
}


void car_start_turning(int direction) {
  if (direction == TURN_LEFT) {
    carState.leftMotor = motor_go(leftMotor, DIR_BW, TURN_SPEED);
    carState.rightMotor = motor_go(rightMotor, DIR_FW, TURN_SPEED);
  } else if (direction == TURN_RIGTH) {
    carState.leftMotor = motor_go(leftMotor, DIR_FW, TURN_SPEED);
    carState.rightMotor = motor_go(rightMotor, DIR_BW, TURN_SPEED);
  }
}

void car_turn(int direction, int amount) {
  car_start_turning(direction);
  delay(amount);
  carState.leftMotor = motor_stop(leftMotor);
  carState.rightMotor = motor_stop(rightMotor);
}


// ==============================
// OBSTACLES
// ==============================

int head_measure_distance(int direction) {
  headservo.write(constrain(direction, HEAD_MIN, HEAD_MAX));
  delay(100);
  int maesuredSum = 0;
  
  for (int i = 0; i < DISTANCE_MEASUREMENT_SAMPLE_COUNT; i++) {
    int dist = sonar.ping_cm();
    bluetooth.println(dist);
    if (dist == 0) {
      dist = MAX_DISTANCE;
    } 
    maesuredSum += dist;
    delay(DISTANCE_MEASUREMENT_SAMPLE_DELAY);
  }

  int result = maesuredSum / DISTANCE_MEASUREMENT_SAMPLE_COUNT;
  bluetooth.print("ad ");
  bluetooth.println(result);
  return result;
}

void obstacle_detection_reset() {
  carState.obstacleDetected = false;
  carState.obstacleSolutionAttempts = 0;
  carState.unableToHandleObstacles = false;
}

void obstacle_handle() {
  carState.obstacleDetected = true;
  carState.obstacleSolutionAttempts++;
  int leftDistance = head_measure_distance(HEAD_LEFT);
  int rightDistance = head_measure_distance(HEAD_RIGHT);
  bluetooth.print("l ");
  bluetooth.print(leftDistance);
  bluetooth.print(" r ");
  bluetooth.print(rightDistance);

  if (max(leftDistance, rightDistance) <= obstacleDist || carState.obstacleSolutionAttempts == MAX_OBSTACLE_DET_ATTEMTPS - 1) {
    //solve by going back a bit
    car_backward(MIN_SPEED);
    delay(1000);
    car_stop();
  } else if (rightDistance > obstacleDist) {
    //go right
    car_turn(TURN_RIGTH, 600);
  } else {
    //go left
    car_turn(TURN_LEFT, 600);
  }
  int distance = head_measure_distance(HEAD_FW);
  if (distance > obstacleDist) {
    carState.obstacleDetected = false;
    carState.obstacleSolutionAttempts = 0;
    bluetooth.println("ohd");
  } 
  //stay in detected obstacle mode, will try to handle it again.
}


// ==============================
// COMMANDS
// ==============================
  


void process_forward() {
  int speed = carState.mainMovementSpeed;
  if (speed == 0) {
    speed = mainMovementSpeedMin;
  } else if (speed > 0 && speed < mainMovementSpeedMax) {
    speed = constrain(speed + mainMovementSpeedIncrement, mainMovementSpeedMin, mainMovementSpeedMax);
  } else if (speed < 0 && speed < -mainMovementSpeedMin) {
    speed = constrain(speed + mainMovementSpeedIncrement, -mainMovementSpeedMax, -mainMovementSpeedMin);
  } else if (speed < 0) { //slowest backward spped
    speed = 0;
  }
  car_move(speed);
  bluetooth.println(speed);
  carState.mainMovementSpeed = speed;
}

void process_backward() {
  int speed = carState.mainMovementSpeed;
  if (speed == 0) {
    speed = -mainMovementSpeedMin;
  } else if (speed < 0 && speed > -mainMovementSpeedMax) {
    speed = constrain(speed - mainMovementSpeedIncrement, -mainMovementSpeedMax, -mainMovementSpeedMin);
  } else if (speed > 0 && speed > mainMovementSpeedMin) {
    speed = constrain(speed - mainMovementSpeedIncrement, mainMovementSpeedMin, mainMovementSpeedMax);
  } else if (speed > 0) { //slowest backward spped
    speed = 0;
  }
  car_move(speed);
  bluetooth.println(speed);
  carState.mainMovementSpeed = speed;
}

void process_left() {
  carState.mainMovementSpeed = 0;
  car_start_turning(TURN_LEFT);
}

void process_right() {
  carState.mainMovementSpeed = 0;
  car_start_turning(TURN_RIGTH);
}

void process_halt() {
  carState.mainMovementSpeed = 0;
  car_stop();
}

// ==============================
// LOOP
// ==============================
  
void loop()
{

    carState.enabled = !digitalRead(ONOFF_PIN);
    if (!carState.enabled) {
      if (carState.leftMotor.speed > 0 || carState.rightMotor.speed > 0) {
        car_stop();
      }
      obstacle_detection_reset();
      carState.autonomousMode = false;
    } else {
      //process command if we have one
      if (bluetooth.available()) { 
        char command = bluetooth.read();
        bluetooth.println(command);
        switch (command) {
          case 'f': //forward
            if (carState.autonomousMode) {
              bluetooth.println("inv");
            } else {
              process_forward();
            }
            break;
          case 'b': //backward
            if (carState.autonomousMode) {
              bluetooth.println("inv");
            } else {
              process_backward();
            }
            break;
          case 'l': //left
            if (carState.autonomousMode) {
              bluetooth.println("inv");
            } else {
              process_left();
            }
            break;
          case 'r': //right
            if (carState.autonomousMode) {
              bluetooth.println("inv");
            } else {
              process_right();
            }
            break;
          case 'x': //halt
            if (carState.autonomousMode) {
              carState.autonomousMode = false;
              obstacle_detection_reset();
            }
            process_halt();
            break;
          case 'a': //autonomous mode toggle
            car_stop();
            obstacle_detection_reset();
            carState.autonomousMode = !carState.autonomousMode;
        }
        bluetooth.println("ok");
      } //if bluetooth.available();
      //write steate to diodes
      digitalWrite(DIODE_RED_PIN, carState.enabled);
      digitalWrite(DIODE_BLUE_PIN, carState.autonomousMode);

      if (carState.autonomousMode)  {
        if (carState.obstacleDetected && carState.obstacleSolutionAttempts < MAX_OBSTACLE_DET_ATTEMTPS) {
          obstacle_handle();
        } else if (carState.obstacleDetected && carState.obstacleSolutionAttempts >= MAX_OBSTACLE_DET_ATTEMTPS) {
          carState.unableToHandleObstacles = true;
          bluetooth.write("fail");
        } else {
          //moving
          int distance = head_measure_distance(HEAD_FW);
          if (distance > obstacleDist) {
            carState.obstacleDetected = false;
            car_forward(AUTO_SPEED);
          } else {
            carState.obstacleDetected = true;
            car_stop();
          }
        }
      } //if autonomousMode

    } //if enabled
    //write steate to diodes
    digitalWrite(DIODE_RED_PIN, carState.enabled);
    digitalWrite(DIODE_BLUE_PIN, carState.autonomousMode);
    // digitalWrite(DIODE_RED_PIN, carState.autonomousMode);
    // if (carState.autonomousMode) {
      
    //   for (int i=100; i < 250; i += 10) {
    //     unsigned long start = millis();
    //     unsigned int left_last_steps = left_step_count;
    //     unsigned int right_last_steps = right_step_count;
    //     car_forward(i);
    //     while (left_step_count - left_last_steps < stepcount * 5) {
    //       delay(1);
    //     }
    //     car_stop();
    //     delay(500);
    //     Serial.print(i);
    //     Serial.print('\t');
    //     Serial.print(millis() - start);
    //     Serial.print('\t');
    //     Serial.print(left_step_count - left_last_steps);
    //     Serial.print(" / ");
    //     Serial.println(right_step_count - right_last_steps);
    //     delay(2000);
    //   }
    //   // for (int i = 100; i <=250; i += 10) {
      //   car_forward(i);
      //   unsigned int last_steps = left_step_count;
      //   delay(500);
      //   unsigned int curr_steps = left_step_count;
      //   unsigned int diff = curr_steps - last_steps;
      //   float rounds = diff / stepcount;
      //   float rpm = rounds * 2 * 60; //per minute
      //   float cmpm = rpm * wheelcircumference * 10; //mm -> cm
      //   Serial.print(i);
      //   Serial.print('\t');
      //   Serial.print(diff);
      //   Serial.print('\t');
      //   Serial.print(rounds);
      //   Serial.print('\t');
      //   Serial.println(cmpm);
      // }
      
    // }


  // digitalWrite(DIODE_BLUE_PIN, carState.obstacleDetected);
  // delay(100);
  
}


void setup() {
  // set all the motor control pins to outputs
  motor_init(leftMotor);
  motor_init(rightMotor);
  headservo.attach(HEADSERVO_PIN);  // attaches the servo on pin 9 to the servo object
  pinMode(OPTO_INTERUPT_RIGHT_PIN, INPUT);
  pinMode(OPTO_INTERUPT_LEFT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(OPTO_INTERUPT_RIGHT_PIN), opto_interrupt, CHANGE);
  attachInterrupt(digitalPinToInterrupt(OPTO_INTERUPT_LEFT_PIN), opto_interrupt, CHANGE);
  Serial.begin(9600); 
  bluetooth.begin(9600);
  pinMode(ONOFF_PIN, INPUT_PULLUP);
  pinMode(DIODE_RED_PIN, OUTPUT);
  pinMode(DIODE_BLUE_PIN, OUTPUT);
  bluetooth.write("setup done");
}

