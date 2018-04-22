#include <Arduino.h>
#include <SPI.h>
#include <RH_NRF24.h>

// Define Joystick Connections
#define JOY_VERT_PIN A0 
#define JOY_HORIZ_PIN A1
#define JOY_BTN_PIN 3
#define REMOTE_MODE_PIN 4
 

struct ButtonDebounce {
  int buttonState;             // the current reading from the input pin
  int lastButtonState = LOW;   // the previous reading from the input pin
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
};

struct CarControlData {
  boolean remoteControlMode;
  byte speedLeft;
  byte speedRight;
  byte direction;
};


// Define Joystick Values - Start at 512 (middle position)
int joyposVert = 512;
int joyposHorz = 512;
int joyBtnPressed = 0;
boolean remoteControlMode = false;

// Singleton instance of the radio driver
RH_NRF24 nrf24;
ButtonDebounce joysticButton;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  while (!Serial) 
    ; // wait for serial port to connect. Needed for Leonardo only
  if (!nrf24.init())
    Serial.println("init failed");
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");   

  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  pinMode(REMOTE_MODE_PIN, OUTPUT);
  digitalWrite(REMOTE_MODE_PIN, LOW);
}

void loop() {
  // Read the Joystick X and Y positions
  joyposVert = analogRead(JOY_VERT_PIN); 
  joyposHorz = analogRead(JOY_HORIZ_PIN);

  //read and debounce button
  int joyBtnReading = !digitalRead(JOY_BTN_PIN);
  
  if (joyBtnReading != joysticButton.lastButtonState) {
    joysticButton.lastDebounceTime = millis();
  }

  if ((millis() - joysticButton.lastDebounceTime) > joysticButton.debounceDelay) {
    if (joyBtnReading != joysticButton.buttonState) {
      joysticButton.buttonState = joyBtnReading;

      if (joysticButton.buttonState == HIGH) {
        remoteControlMode = !remoteControlMode;
      }
    }
  }

  joysticButton.lastButtonState = joyBtnReading;

  digitalWrite(REMOTE_MODE_PIN, remoteControlMode);

  Serial.print(joyBtnReading * 512);
  Serial.print("\t");
  Serial.print(joysticButton.buttonState * 510);
  Serial.print("\t");
  Serial.print(remoteControlMode * 508);
  Serial.print("\t");
  Serial.print(joyposVert);
  Serial.print("\t");
  Serial.println(joyposHorz);

//   if (nrf24.available())
//   {
//     // Should be a message for us now   
//     uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
//     uint8_t len = sizeof(buf);
//     if (nrf24.recv(buf, &len))
//     {
// //      NRF24::printBuffer("request: ", buf, len);
//       Serial.println((char*)buf);
      
//       // Send a reply
//     //   uint8_t data[] = "And hello back to you";
//     //   nrf24.send(data, sizeof(data));
//     //   nrf24.waitPacketSent();
//     //   Serial.println("Sent a reply");
//     }
//     else
//     {
//       Serial.println("recv failed");
//     }
//   }
}