#include <Arduino.h>
#include <Servo.h>
#include <NewPing.h>
#include <constants.h>
#include <SPI.h>
#include <RH_NRF24.h>


struct MotorConfig {
  int pwmPin;
  int forwardPin;
  int backwardPin;
  MotorConfig(int pwm, int fw, int bw) : pwmPin(pwm), forwardPin(fw), backwardPin(bw) {}
};

struct CarState {
  boolean autonomousMode = false;
  boolean obstacleDetected = false;
  int obstacleSolutionAttempts = 0;
};

const int obstacleDist = 50;

RH_NRF24 radio;
CarState carState;

MotorConfig leftMotor = MotorConfig(3, 4, 2);
MotorConfig rightMotor = MotorConfig(5, 7, 6);

Servo headservo;  // create servo object to control a servo
int pos = 0;    // variable to store the servo position
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);


void motor_init(MotorConfig &motor) {
  pinMode(motor.pwmPin, OUTPUT);
  pinMode(motor.forwardPin, OUTPUT);
  pinMode(motor.backwardPin, OUTPUT);
}

void motor_go(MotorConfig &motor, int dir, int speed) {
  digitalWrite(motor.forwardPin, dir == DIR_FW ? HIGH : LOW);
  digitalWrite(motor.backwardPin, dir == DIR_FW ? LOW : HIGH);
  analogWrite(motor.pwmPin, speed);
}

void motor_stop(MotorConfig &motor) {
  digitalWrite(motor.forwardPin, LOW);
  digitalWrite(motor.backwardPin, LOW);
  analogWrite(motor.pwmPin, 0);
}


void setup() {
  // set all the motor control pins to outputs
  motor_init(leftMotor);
  motor_init(rightMotor);
  headservo.attach(HEADSERVO_PIN);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); 
  pinMode(ONOFF_PIN, INPUT_PULLUP);
  pinMode(DIODE_RED_PIN, OUTPUT);
  pinMode(DIODE_BLUE_PIN, OUTPUT);
  if (!radio.init()) {
    Serial.println("init failed");
  }
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!radio.setChannel(1)) {
    Serial.println("setChannel failed");
  }
  if (!radio.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
    Serial.println("setRF failed");    
  }
}

void radio_send(String s) {
  Serial.println(String("Sending: " + s));
  char data[s.length() + 1];
  s.toCharArray(data, s.length()+ 1) ;
  radio.send(data, sizeof(data));
  radio.waitPacketSent();
}


int head_measure_distance(int direction) {
  radio_send(String("Move servo to: ") + direction);
  headservo.write(constrain(direction, HEAD_MIN, HEAD_MAX));
  delay(250);
  //Serial.print("Moved servo to: ");
  //Serial.println(headservo.read());
  int maesuredSum = 0;
  
  for (int i = 0; i < DISTANCE_MEASUREMENT_SAMPLE_COUNT; i++) {
    int dist = sonar.ping_cm();
    radio_send(String("measured: ") + dist);
    if (dist == 0) {
      dist = MAX_DISTANCE;
    } 
    maesuredSum += dist;
    delay(DISTANCE_MEASUREMENT_SAMPLE_DELAY);
  }

  int result = maesuredSum / DISTANCE_MEASUREMENT_SAMPLE_COUNT;
  radio_send(String("avg: ") + result);
  return result;
}

void car_forward(int speed) {
  motor_go(leftMotor, DIR_FW, speed);
  motor_go(rightMotor, DIR_FW, speed);
}

void cara_backward(int speed) {
  motor_go(leftMotor, DIR_BW, speed);
  motor_go(rightMotor, DIR_BW, speed);
}

void car_stop() {
  motor_stop(leftMotor);
  motor_stop(rightMotor);
}



void car_turn(int direction, int amount) {
  if (direction == TURN_LEFT) {
    motor_go(leftMotor, DIR_BW, TURN_SPEED);
    motor_go(rightMotor, DIR_FW, TURN_SPEED);
  } else if (direction == TURN_RIGTH) {
    motor_go(leftMotor, DIR_FW, TURN_SPEED);
    motor_go(rightMotor, DIR_BW, TURN_SPEED);
  }
  delay(amount);
  motor_stop(leftMotor);
  motor_stop(rightMotor);
}


void obstacle_handle() {
  radio_send(String("Trying to hadle obstacle"));
  carState.obstacleDetected = true;
  carState.obstacleSolutionAttempts++;
  int leftDistance = head_measure_distance(HEAD_LEFT);
  int rightDistance = head_measure_distance(HEAD_RIGHT);
  radio_send(String("left\t") + leftDistance + String("\tright\t") + rightDistance);

  if (max(leftDistance, rightDistance) <= obstacleDist || carState.obstacleSolutionAttempts == MAX_OBSTACLE_DET_ATTEMTPS - 1) {
    //solve by going back a bit
    cara_backward(MIN_SPEED);
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
    radio_send("OBstacle handled");
  } 
  //stay in detected obstacle mode, will try to handle it again.
}


void loop()
{
  

  carState.autonomousMode = !digitalRead(ONOFF_PIN);
  digitalWrite(DIODE_RED_PIN, carState.autonomousMode);
  if (carState.autonomousMode)  {
    // car_forward(MIN_SPEED);
    // delay(1000);
    // car_stop();
    // car_turn(TURN_LEFT, 700);
    // car_forward(MIN_SPEED);
    // delay(1000);
    // car_stop();
    // car_turn(TURN_RIGTH, 700);

    if (carState.obstacleDetected && carState.obstacleSolutionAttempts < MAX_OBSTACLE_DET_ATTEMTPS) {
      obstacle_handle();
    } else if (carState.obstacleDetected && carState.obstacleSolutionAttempts >= MAX_OBSTACLE_DET_ATTEMTPS) {
      //error mode -> just blink with diode
      radio_send("Unable to hadle obstacle");
      digitalWrite(DIODE_BLUE_PIN, HIGH);
      delay(500);
      digitalWrite(DIODE_BLUE_PIN, LOW);
      delay(500);
    } else {
      //moving
      int distance = head_measure_distance(HEAD_FW);
      if (distance > obstacleDist) {
        carState.obstacleDetected = false;
        car_forward(MIN_SPEED);
      } else {
        carState.obstacleDetected = true;
        car_stop();

      }
    }
    digitalWrite(DIODE_BLUE_PIN, carState.obstacleDetected);
  } else {
    //stop and reset state
    carState.obstacleDetected = false;
    carState.obstacleSolutionAttempts = 0;
    car_stop();
    radio_send("idle");
  }
  digitalWrite(DIODE_BLUE_PIN, carState.obstacleDetected);
  delay(100);
}