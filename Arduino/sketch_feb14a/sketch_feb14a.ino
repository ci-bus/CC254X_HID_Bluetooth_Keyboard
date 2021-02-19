#include <CC254XBHID.h>

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

CC254XBHID bhid(2, 3); //2 rx, 3 tx

bool wasPressed = false;
uint8_t battery = 100;

void setup() {
  /*mySerial.begin(9600);
  Serial.begin(9600);
  while (!Serial) { ; }
  */
  Serial.begin(57600);
  bhid.begin(57600);
  // Pin to GND to test key
  pinMode(8, INPUT);
  digitalWrite(8, HIGH);
}

int test = 1;

void loop() {

  bool pressed = digitalRead(8) == LOW;
  if (pressed != wasPressed) {
    wasPressed = pressed;
    
    if (test == 0) {
      if (!pressed) {
        bhid.sendKey(0x1e);
        bhid.sendKey(0x1f);
        bhid.sendKey(0x20);
        bhid.sendKey(0x21);
        bhid.sendKey(0x22);
        bhid.sendKey(0x23);
      }
    } 
    
    if (test == 1) {
      bhid.sendKey(pressed, 6);
    }
    
    if (test == 2 && battery > 0) {
      if (!pressed) {
        bhid.setBattery(battery--);
      }
    }
    
    if (test == 3) {
      bhid.sendConsumerKey(pressed, 64, 0x00);     
    }

    if (!pressed) {
        delay(50);
    }
  }
  
  if (int data = bhid.loop()) {
    Serial.write(data);
  }

  if (Serial.available()) {
    bhid.write(Serial.read());
  }
}
