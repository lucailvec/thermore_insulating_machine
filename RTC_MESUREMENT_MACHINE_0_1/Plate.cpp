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
  if(Plate::STATE::OFF==_s)
    analogWrite(_numPin,pwm);
  else
    analogWrite(_numPin,0);
}

void Plate::turn(STATE s){
  _s= s;
}

