#ifndef CC254X_HID
#define CC254X_HID

#include <AltSoftSerial.h>

class CC254XBHID: public AltSoftSerial
{
  public:
    CC254XBHID(const byte rx = 2, const byte tx = 3) : AltSoftSerial(rx, tx){};
    // Press and release keyboard keys
    void sendKey(uint8_t keyCode);
    // Press or release keyboard keys
    void sendKey(bool pressed, uint8_t keyCode);
    // Press and release consumer keyboard keys
    void sendConsumerKey(uint16_t keyCode);
    // Press or release consumer keyboard keys
    void sendConsumerKey(bool pressed, uint16_t keyCode);
    // Press or release consumer keyboard keys
    void sendConsumerKey(bool pressed, uint8_t loByte, uint8_t hiByte);
    // Send actions in buffer
    void processBuffer();
    // Loop to detect feedback of cc254x
    int loop();
    // Set battery level porcent
    void setBattery(uint8_t level);
  private:
    void sendBuffer(char buf[8]);
    bool stateFree = true;
    char buffer[24][8];
    uint8_t bufferStart = 0;
    uint8_t bufferEnd = 0;
    uint8_t safeWait = 30;
};

#endif // CC254X_HID