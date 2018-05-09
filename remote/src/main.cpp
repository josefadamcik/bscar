#include <Arduino.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RHReliableDatagram.h>
#include <../../shared/shared_structs.h>


// Define Joystick Connections
#define JOY_VERT_PIN A0 
#define JOY_HORIZ_PIN A1
#define JOY_BTN_PIN 3
#define REMOTE_MODE_PIN 4
 

struct ButtonWDebounce {
  int pin;
  boolean negated = false; //true for button which brings signal to ground when pressed.
  int buttonState = LOW;             // the current reading from the input pin
  int lastButtonState = LOW;   // the previous reading from the input pin
  unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
  unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
  ButtonWDebounce(int pin, boolean negated) : pin(pin), negated(negated) {}
};



int joyposVert = 512;
int joyposHorz = 512;
int joyBtnPressed = 0;
boolean remoteControlMode = false;

RH_NRF24 nrf24;
RHReliableDatagram radio(nrf24, REMOTE_ADDRESS);
ButtonWDebounce joysticButton = ButtonWDebounce(JOY_BTN_PIN, true);

/**
 * @return true when button's value was changed
 */
boolean button_check_debounced(ButtonWDebounce &button) {
  int reading = digitalRead(button.pin);
  boolean changeTriggered = false;
  if (button.negated) {
    reading = !reading;
  }
  
  if (reading != button.lastButtonState) {
    button.lastDebounceTime = millis();
  }

  if ((millis() - button.lastDebounceTime) > button.debounceDelay) {
    if (reading != button.buttonState) {
      button.buttonState = reading;
      changeTriggered = true;
    }
  }

  button.lastButtonState = reading;
  return changeTriggered;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!radio.init()) {
    Serial.println("init failed");
  }

  radio.setTimeout(500);
  radio.setRetries(5);

  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(2)) {
     Serial.println("setChannel failed");
  }
    
  if (!nrf24.setRF(RH_NRF24::DataRate1Mbps, RH_NRF24::TransmitPowerm18dBm )) {
     Serial.println("setRF failed");   
  }
    

  pinMode(JOY_BTN_PIN, INPUT_PULLUP);
  pinMode(REMOTE_MODE_PIN, OUTPUT);
  digitalWrite(REMOTE_MODE_PIN, LOW);
}

void loop() {
  // Read the Joystick X and Y positions
  joyposVert = analogRead(JOY_VERT_PIN); 
  joyposHorz = analogRead(JOY_HORIZ_PIN);
  boolean trigger = false;

  if (button_check_debounced(joysticButton)) {
    if (joysticButton.buttonState == HIGH) {
      remoteControlMode = !remoteControlMode;
      trigger = true;
    }
  }
  
  digitalWrite(REMOTE_MODE_PIN, remoteControlMode);

  // Serial.print(joysticButton.buttonState * 510);
  // Serial.print("\t");
  // Serial.print(remoteControlMode * 508);
  // Serial.print("\t");
  // Serial.print(joyposVert);
  // Serial.print("\t");
  // Serial.println(joyposHorz);

  if (trigger) {
    Serial.println("triggered, send command");
    uint8_t command[3];
    if (remoteControlMode) {
      command[0] = COMMAND_GO;
      command[1] = DIR_FW;
      command[2] = 160;
    } else {
      command[0] = COMMAND_STOP;
      command[1] = command[2] = 0;
    }
    Serial.print(command[0]);
    Serial.print(command[1]);
    Serial.println(command[2]);
   //Send a message containing Joystick data to manager_server
    if (radio.sendtoWait(command, sizeof(command), CAR_ADDRESS))
    {
      CommandResponse data;
      // Now wait for a reply from the server
      uint8_t len;
      uint8_t from;
      if (radio.recvfromAckTimeout((uint8_t *)&data, &len, 3000, &from))
      {
          Serial.print("recived: ");
          Serial.print(len);
          Serial.print(" b, from: ");
          Serial.print(from);
          Serial.println();
          Serial.println(data.state);
          // Serial.print("Left motor ");
          // Serial.print(data.leftMotor.direction);
          // Serial.print(' ');
          // Serial.print(data.leftMotor.speed);
          // Serial.println();
          // Serial.print("Right motor ");
          // Serial.print(data.rightMotor.direction);
          // Serial.print(' ');
          // Serial.print(data.rightMotor.speed);
          // Serial.println();
          // Serial.print("autonomous?: ");
          // Serial.print(data.autonomousMode);
          Serial.println();
      }
      else
      {
        Serial.println("No reply, is nrf24_reliable_datagram_server running?");
      }
    }
    else {
      Serial.println("sendtoWait failed");
    }

  }

  // if (radio.available()) {
  //   CarState data;
  //   uint8_t len;
  //   uint8_t from;
  //   uint8_t to;
  //   if (radio.recvfrom((uint8_t *)&data, &len, &from, &to, NULL, NULL)) {
  //       // Do something with data.name and data.skill
  //       Serial.print("recived: ");
  //       Serial.print(len);
  //       Serial.print(' ');
  //       Serial.print(from);
  //       Serial.println();
  //       Serial.print("Left motor ");
  //       Serial.print(data.leftMotor.direction);
  //       Serial.print(' ');
  //       Serial.print(data.leftMotor.speed);
  //       Serial.println();
  //       Serial.print("Right motor ");
  //       Serial.print(data.rightMotor.direction);
  //       Serial.print(' ');
  //       Serial.print(data.rightMotor.speed);
  //       Serial.println();
  //       Serial.print("autonomous?: ");
  //       Serial.print(data.autonomousMode);
  //       Serial.println();
  //   }
  // }
}