
#include "Fridge.h"
#ifndef Arduino_H
#include "Arduino.h"
#endif

Fridge::Fridge(int pin){
  pinMode(_numPin,OUTPUT);
  _onState = HIGH;
}

void Fridge::turn(State st){
  if(st==ON)
    digitalWrite(_numPin,(_onState)?HIGH:LOW);
  else
    digitalWrite(_numPin,(_onState)?LOW:HIGH);
  
}

