/*

#ifndef "BUTTON_H"
#define "BUTTON_H"

class Button{
  public:
    Button(int *val);
    Button (int *val, int interval);
    bool getState();
  private:
    int * _valPointer;
    int _interval;
    unsigned long _lastRead;
    bool _lastState;
}

#endif
 */

#include "Button.h"

#ifndef ARDUINO_H
#include "Arduino.h"
#endif

 Button::Button(int pin){
  _pin = pin;
  pinMode(_pin,INPUT_PULLUP);
  _interval = 0;
  _lastRead=0;
 }

 Button::Button(int pin, int interval){
  _pin = pin;
  pinMode(_pin,INPUT_PULLUP);
  _interval = interval;
  _lastRead=0;
 }

Button::STATE  Button::getState(){
  if(_interval <= millis() - _lastRead){
    _lastRead = millis();
    if( digitalRead(_pin) == LOW){ //vuol dire che è premuto
            return Button::STATE::DOWN;
    }
  } 
  return Button::STATE::UP;
    //return digitalRead(_pin);
}

