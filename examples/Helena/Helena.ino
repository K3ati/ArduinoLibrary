#include <TactTiles.h>
#include <Wire.h>

class Glove : public TTDevice {
public:
  Glove(uint8_t in, uint8_t out):TTDevice(in, out){};

public:

  void inputDetected (uint32_t input){
    
  }

  void symbolParsed (int16_t id){
    
  }

  void messageReceived(const uint8_t * data, uint8_t size){
    
  }
};

Glove glove(16,16);

void setup() {
  glove.begin();
}

void loop() {
  glove.step();
}
