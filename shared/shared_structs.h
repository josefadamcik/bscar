#define REMOTE_ADDRESS 1
#define CAR_ADDRESS 2

#define DIR_FW 1 
#define DIR_NONE 0
#define DIR_BW -1 

#define DIST_ANGLE_COUNT 9

const byte DistanceAngles[] = {5, 10, 15, 80, 90 , 100, 165, 170, 175};

struct MotorState {
  byte speed;
  byte direction;
  MotorState() : speed(0), direction(DIR_NONE) {};
  MotorState(byte speed, byte direction) : speed(speed), direction(direction) {};
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
};

