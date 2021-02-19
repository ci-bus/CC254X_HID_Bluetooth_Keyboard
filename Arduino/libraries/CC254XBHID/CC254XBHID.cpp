/*
  CC254XBHID.cpp - Library for control CC254X with HID Firmware
  Created by Miguel Ángel Calero 16 Feb 2021
*/

#include <Arduino.h>
#include <string.h>
#include <CC254XBHID.h>

void CC254XBHID::processBuffer()
{
  stateFree = true;
  if (bufferStart < bufferEnd) {
    char buf[8];
    sprintf(buf, "%s", buffer[bufferStart]);
    bufferStart++;
    // All buffer processed
    if (bufferStart == bufferEnd) {
      bufferStart = 0;
      bufferEnd = 0;
    }
    // Send buffer saved
    delay(safeWait);
    this->sendBuffer(buf);
  }
}

void CC254XBHID::sendBuffer(char buf[8])
{
  if (stateFree) {
    stateFree = false;
    this->write(buf);
  } else {
    sprintf(buffer[bufferEnd], "%s", buf);
    bufferEnd++;
  }
}

void CC254XBHID::sendKey(uint8_t keyCode)
{
  char buf[8];
  sprintf(buf, "%s%c\n", "KPR", keyCode);
  this->sendBuffer(buf);
}

void CC254XBHID::sendKey(bool pressed, uint8_t keyCode)
{
  char buf[8];
  sprintf(buf, "%s%c\n", pressed ? "KP" : "KR", keyCode);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(uint16_t keyCode) {
  byte lo, hi;
  lo = lowByte(keyCode);
  hi = highByte(keyCode);
  char buf[8];
  sprintf(buf, "%s%c%c\n", "CPR", lo, hi);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(bool pressed, uint16_t keyCode) {
  byte lo, hi;
  lo = lowByte(keyCode);
  hi = highByte(keyCode);
  char buf[8];
  sprintf(buf, "%s%c%c\n", pressed ? "CP" : "CR", lo, hi);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(bool pressed, uint8_t loByte, uint8_t hiByte) {
  char buf[8];
  sprintf(buf, "%s%c%c\n", pressed ? "CP" : "CR", loByte, hiByte);
  this->sendBuffer(buf);
}