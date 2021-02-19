/*
  CC254XBHID.cpp - Library for control CC254X with HID Firmware
  Created by Miguel Angel Calero 16 Feb 2021
*/

/*
Command sets, chosen options need to be stored in non-volatile memory
- SN,<name>  + set device name
- S,R Reset the device
- S,ID print ID of module
- S,N print the current Bluetooth name
- S,D Set device to be discoverable
- S,DT  Disconnect device from host
- S,S Sleed mode disable
- S,A Sleed mode enable
- S,F Notificate key actions and others
- BL<porcent> Level of battery
- KP<keyCode> Press a key
- KR<keyCode> Release a key
- KPR<keyCode> Press ans release a key
- CP<lowByte><highByte> Press consumer key
- CR<lowByte><highByte> Release consumer key
- CPR<lowByte><highByte> Press and release consumer key
*/

#include <Arduino.h>
#include <string.h>
#include <CC254XBHID.h>

int CC254XBHID::loop()
{
  if (this->available()) {
    this->processBuffer();
    return this->read();   
  }
  return 0;
}

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
    this->println(buf);
  } else {
    sprintf(buffer[bufferEnd], "%s", buf);
    bufferEnd++;
  }
}

void CC254XBHID::sendKey(uint8_t keyCode)
{
  char buf[8];
  sprintf(buf, "%s%c", "KPR", keyCode);
  this->sendBuffer(buf);
}

void CC254XBHID::sendKey(bool pressed, uint8_t keyCode)
{
  char buf[8];
  sprintf(buf, "%s%c", pressed ? "KP" : "KR", keyCode);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(uint16_t keyCode) {
  byte lo, hi;
  lo = lowByte(keyCode);
  hi = highByte(keyCode);
  char buf[8];
  sprintf(buf, "%s%c%c", "CPR", lo, hi);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(bool pressed, uint16_t keyCode) {
  byte lo, hi;
  lo = lowByte(keyCode);
  hi = highByte(keyCode);
  char buf[8];
  sprintf(buf, "%s%c%c", pressed ? "CP" : "CR", lo, hi);
  this->sendBuffer(buf);
}

void CC254XBHID::sendConsumerKey(bool pressed, uint8_t loByte, uint8_t hiByte) {
  char buf[8];
  sprintf(buf, "%s%c%c", pressed ? "CP" : "CR", loByte, hiByte);
  this->sendBuffer(buf);
}

void CC254XBHID::setBattery(uint8_t level) {
  char buf[8];
  sprintf(buf, "%s%c", "BL", level);
  this->sendBuffer(buf);
}