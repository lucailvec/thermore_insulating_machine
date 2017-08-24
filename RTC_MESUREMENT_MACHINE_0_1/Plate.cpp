#ifndef Plate_H
#include "Plate.h"
#endif

Plate::Plate(int pin,int maxPWM){
  _numPin=pin;
  pinMode(_numPin,OUTPUT);
  _maxPWM = maxPWM;
 // analogWrite(_numPin,0); 
}

void Plate::set(int pwm){
  /*if(pwm>=_maxPWM)
    analogWrite(_numPin,_maxPWM);
  else if (pwm<0)
    analogWrite(_numPin,0);
  else*/
    analogWrite(_numPin,pwm);
}

