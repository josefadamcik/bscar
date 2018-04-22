#include <Arduino.h>
#include <SPI.h>
#include <RH_NRF24.h>
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
  if (!nrf24.init()) {
    Serial.println("init failed");
  }
  
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  if (!nrf24.setChannel(1)) {
    Serial.println("setChannel failed");
  }
    
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm)) {
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

  if (button_check_debounced(joysticButton)) {
    if (joysticButton.buttonState == HIGH) {
      remoteControlMode = !remoteControlMode;
    }
  }
  
  digitalWrite(REMOTE_MODE_PIN, remoteControlMode);

  Serial.print(joysticButton.buttonState * 510);
  Serial.print("\t");
  Serial.print(remoteControlMode * 508);
  Serial.print("\t");
  Serial.print(joyposVert);
  Serial.print("\t");
  Serial.println(joyposHorz);
}