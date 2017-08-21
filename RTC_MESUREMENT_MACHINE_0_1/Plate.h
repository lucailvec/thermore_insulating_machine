
#ifndef Plate_H
#define Plate_H

#ifndef Arduino_H
#include "Arduino.h"
#endif

class Plate
{
public: 
  Plate(int pin, int maxPWM);
  void set(int pwm);
private:
  int _numPin;
  bool _maxPWM;
  
};

#endif
