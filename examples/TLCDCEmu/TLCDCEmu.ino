#include <TLCDCEmu.h>

TLCDCEmu emulator;

void setup(){
  delay(10);
  emulator.init();
}

void loop(){
  emulator.talk();
}