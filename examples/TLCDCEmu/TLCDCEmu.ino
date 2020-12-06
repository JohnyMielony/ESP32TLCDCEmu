#include <TLCDCEmu.h>

TLCDCEmu emulator;

void setup(){
  delay(3000);
  emulator.init();
  delay(2000);
}

void loop(){
  emulator.talk();
}