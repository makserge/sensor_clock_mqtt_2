#include "SunFounderEmo.h"
#include <SPI.h>

SunFounderEmo::SunFounderEmo(byte csPin) {
  CS_PIN = csPin;

  pinMode(CS_PIN, OUTPUT);
  
  SPI.begin();
  
  clearDisplay();
}

void SunFounderEmo::resetDisplay() {
  memcpy(ledData, DIGIT_SWITCH_OFF, 24);
}

void SunFounderEmo::clearDisplay() {
  memcpy(ledData, DIGIT_SWITCH_OFF, 24);
  updateDisplay();
}

void SunFounderEmo::setDigit(byte digit, byte value) {
  byte data[24];
  
  switch (digit) {
    case 0:
      memcpy(data, DIGIT0[value], 24);
      break;
    case 1:
      memcpy(data, DIGIT1[value], 24);
      break;
    case 2:
      memcpy(data, DIGIT2[value], 24);
      break;
    case 3:
      memcpy(data, DIGIT3[value], 24);
      break;
  }
  setData(data);
}

void SunFounderEmo::showTimeTick(boolean isShown) {
  byte data[24];
  memcpy(data, isShown ? TIME_SEMICOLON : DIGIT_SWITCH_OFF, 24);
  
  setData(data);
}

void SunFounderEmo::setData(byte data[24]) {
  byte i;
  for (i = 0; i < 24; i++) {
    ledData[i] = data[i] | ledData[i];
  }
}

void SunFounderEmo::sendByte(byte data) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(data);  
  digitalWrite(CS_PIN, HIGH);
}

void SunFounderEmo::updateDisplay() {
  byte i;

  sendByte(START_ADDRESS);
  for (i = 0; i < 24; i++) {
    sendByte(ledData[i]);  
  }
}
