#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include <constants.h>
#include <SPI.h>
#include <../../shared/shared_structs.h>


struct MotorConfig {
  int pwmPin;
  int forwardPin;
  int backwardPin;
  MotorConfig(int pwm, int fw, int bw) : pwmPin(pwm), forwardPin(fw), backwardPin(bw) {}
};


const int obstacleDist = 50;

CarState carState;

//with 5 and 6 for pwm the both sides behaves the same, with 3&5 it seemed that one side (pin 3) had lower voltage
MotorConfig leftMotor = MotorConfig(5, 7, 4);
MotorConfig rightMotor = MotorConfig(6, 8, 10);


Servo headservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


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

volatile unsigned int left_step_count = 0;
volatile unsigned int right_step_count = 0;
volatile byte left_last_state = LOW;
volatile byte right_last_state = LOW;
const float stepcount = 20.00;  // 20 Slots in disk, change if different
const float wheeldiameter = 66.10; // Wheel diameter in millimeters, change if different
const float wheelcircumference = wheeldiameter * 3.1416;
 

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
  pinMode(ONOFF_PIN, INPUT_PULLUP);
  pinMode(DIODE_RED_PIN, OUTPUT);
  pinMode(DIODE_BLUE_PIN, OUTPUT);
}


int head_measure_distance(int direction) {
  Serial.println(String("Move servo to: ") + direction);
  headservo.write(constrain(direction, HEAD_MIN, HEAD_MAX));
  delay(250);
  //Serial.print("Moved servo to: ");
  //Serial.println(headservo.read());
  int maesuredSum = 0;
  
  for (int i = 0; i < DISTANCE_MEASUREMENT_SAMPLE_COUNT; i++) {
    int dist = sonar.ping_cm();
    Serial.println(String("measured: ") + dist);
    if (dist == 0) {
      dist = MAX_DISTANCE;
    } 
    maesuredSum += dist;
    delay(DISTANCE_MEASUREMENT_SAMPLE_DELAY);
  }

  int result = maesuredSum / DISTANCE_MEASUREMENT_SAMPLE_COUNT;
  Serial.println(String("avg: ") + result);
  return result;
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

void car_turn(int direction, int amount) {
  if (direction == TURN_LEFT) {
    carState.leftMotor = motor_go(leftMotor, DIR_BW, TURN_SPEED);
    carState.rightMotor = motor_go(rightMotor, DIR_FW, TURN_SPEED);
  } else if (direction == TURN_RIGTH) {
    carState.leftMotor = motor_go(leftMotor, DIR_FW, TURN_SPEED);
    carState.rightMotor = motor_go(rightMotor, DIR_BW, TURN_SPEED);
  }
  delay(amount);
  carState.leftMotor = motor_stop(leftMotor);
  carState.rightMotor = motor_stop(rightMotor);
}


void obstacle_handle() {
  Serial.println(String("Trying to hadle obstacle"));
  carState.obstacleDetected = true;
  carState.obstacleSolutionAttempts++;
  int leftDistance = head_measure_distance(HEAD_LEFT);
  int rightDistance = head_measure_distance(HEAD_RIGHT);
  Serial.println(String("left\t") + leftDistance + String("\tright\t") + rightDistance);

  if (max(leftDistance, rightDistance) <= obstacleDist || carState.obstacleSolutionAttempts == MAX_OBSTACLE_DET_ATTEMTPS - 1) {
    //solve by going back a bit
    car_backward(120);
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
    Serial.println("OBstacle handled");
  } 
  //stay in detected obstacle mode, will try to handle it again.
}

void process_go(uint8_t direction, uint8_t speed) {
  if (direction == DIR_FW) {
    car_forward(speed);
  } else if (direction == DIR_BW) {
    car_backward(speed);
  } else {
    car_stop();
  }
}



void loop()
{

  // carState.autonomousMode = !digitalRead(ONOFF_PIN);
  // digitalWrite(DIODE_RED_PIN, carState.autonomousMode);
  // if (carState.autonomousMode)  {
  //   if (carState.obstacleDetected && carState.obstacleSolutionAttempts < MAX_OBSTACLE_DET_ATTEMTPS) {
  //     obstacle_handle();
  //   } else if (carState.obstacleDetected && carState.obstacleSolutionAttempts >= MAX_OBSTACLE_DET_ATTEMTPS) {
  //     //error mode -> just blink with diode
  //     Serial.println("Unable to hadle obstacle");
  //     digitalWrite(DIODE_BLUE_PIN, HIGH);
  //     delay(500);
  //     digitalWrite(DIODE_BLUE_PIN, LOW);
  //     delay(500);
  //   } else {
  //     //moving
  //     int distance = head_measure_distance(HEAD_FW);
  //     if (distance > obstacleDist) {
  //       carState.obstacleDetected = false;
  //       car_forward(150);
  //     } else {
  //       carState.obstacleDetected = true;
  //       car_stop();

  //     }
  //   }
  //   digitalWrite(DIODE_BLUE_PIN, carState.obstacleDetected);
  // } else {
  //   //stop and reset state
  //   carState.obstacleDetected = false;
  //   carState.obstacleSolutionAttempts = 0;
  //   car_stop();
  // }
    carState.autonomousMode = !digitalRead(ONOFF_PIN);
    digitalWrite(DIODE_RED_PIN, carState.autonomousMode);
    if (carState.autonomousMode) {
      
      for (int i=100; i < 250; i += 10) {
        unsigned long start = millis();
        unsigned int left_last_steps = left_step_count;
        unsigned int right_last_steps = right_step_count;
        car_forward(i);
        while (left_step_count - left_last_steps < stepcount * 5) {
          delay(1);
        }
        car_stop();
        delay(500);
        Serial.print(i);
        Serial.print('\t');
        Serial.print(millis() - start);
        Serial.print('\t');
        Serial.print(left_step_count - left_last_steps);
        Serial.print(" / ");
        Serial.println(right_step_count - right_last_steps);
        delay(2000);
      }
      // for (int i = 100; i <=250; i += 10) {
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
      
    }


  // digitalWrite(DIODE_BLUE_PIN, carState.obstacleDetected);
  // delay(100);
  
}
