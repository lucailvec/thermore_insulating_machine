
#ifndef Plate_H
#define Plate_H

#ifndef Arduino_H
#include "Arduino.h"
#endif

class Plate
{
public: 
  enum STATE { ON, OFF };
  Plate(int pin, int maxPWM);
  void set(int pwm);
  void turn(STATE s);
private:
  STATE _s;
  int _numPin;
  bool _maxPWM;
  
};

#endif
