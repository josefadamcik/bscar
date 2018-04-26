
#define TRIGGER_PIN  A3  // Arduino pin tied to trigger pin on the ultrasonic sensor.
#define ECHO_PIN     A3  // Arduino pin tied to echo pin on the ultrasonic sensor.
#define MAX_DISTANCE 400 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.
#define HEADSERVO_PIN 9 //note PWM on 9 and 10 is disabled by servor library
#define ONOFF_PIN A0
#define DIODE_RED_PIN A1
#define DIODE_BLUE_PIN A2

#define TURN_LEFT -1
#define TURN_RIGTH 1
#define HEAD_MIN 0
#define HEAD_MAX 180
#define HEAD_LEFT 165
#define HEAD_RIGHT 15
#define HEAD_FW 89
#define HEAD_ANGLE_CORRECTION -1;

#define MIN_SPEED 100
#define TURN_SPEED 150

#define DISTANCE_MEASUREMENT_SAMPLE_COUNT 3
#define DISTANCE_MEASUREMENT_SAMPLE_DELAY 30

#define MAX_OBSTACLE_DET_ATTEMTPS 5