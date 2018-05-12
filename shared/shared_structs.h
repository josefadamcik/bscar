#define DIST_ANGLE_COUNT 9

/*
* Communication protocol.
* In a nutshell: remote sends commands and gets different responses back. Response is a struct containing data.
*/

/** Send command for movement, paramms: direction(byte), speed(byte), expects CarState as response */
const uint8_t COMMAND_GO = 1;
/** Send command to stop, expects CarState as response */
const uint8_t COMMAND_STOP = 2;
/** Send command for turning, paramms: direction(byte), howmuch(byte), expects CarState as response */
const uint8_t COMMAND_TURN = 3;
/** Set params for car, params: TBD, expects ParamsState as response */
const uint8_t COMMAND_PARAMS = 4;
/** Requests specific state response, parameter: what(byte), see GET_STATE_xxx. Expects chosen data as response */
const uint8_t COMMAND_GET = 5;

const uint8_t GET_STATE_CAR = 1;
const uint8_t GET_STATE_PARAMS = 2;
const uint8_t GET_STATE_DISTANCE = 3;

const uint8_t DIR_FW = 1; 
const uint8_t DIR_NONE = 0;
const uint8_t DIR_BW = -1;

const uint8_t TURN_LEFT = 1;
const uint8_t TURN_RIGTH = -1;


const byte DistanceAngles[] = {5, 10, 15, 80, 90 , 100, 165, 170, 175};

struct MotorState {
  uint8_t speed;
  uint8_t direction;
  MotorState() : speed(0), direction(DIR_NONE) {};
  MotorState(uint8_t speed, uint8_t direction) : speed(speed), direction(direction) {};
};


struct DistanceSensorState {
  //3 for left, 3 for middle, 3 for right
  unsigned int distances[DIST_ANGLE_COUNT];
  //timestamp for each angle
  unsigned long timestamp[DIST_ANGLE_COUNT];
};

struct CarControlData {
  boolean remoteControlMode;
  MotorState leftMotor;
  MotorState rightMotor;
};


struct CarState {
  boolean autonomousMode = false;
  boolean obstacleDetected = false;
  int obstacleSolutionAttempts = 0;
  MotorState leftMotor;
  MotorState rightMotor;
  int mainMovementSpeed = 0;
};


struct CommandResponse {
  uint8_t state;
};
