
#include "Measurable.h"

#ifndef MATH_H
#include <math.h>
#endif

#ifndef ARDUINO_H
#include <Arduino.h>
#endif


  //float _ar[5];
  //unsigned int _counter;

#include "QuickStats.h"


Measurable::Measurable(long interval,double * ar,int num){
  _interval=interval;
  _counter = 0;
  _ar=ar;
  NUMSAMPLES=num;
  _stats = new QuickStats();
}
double Measurable::median(){
  return _stats->median((float *)_ar,NUMSAMPLES);
}
double Measurable::median(int num){
  return _stats->median((float *)_ar,num);
}
double Measurable::average(){
  return _stats->average((float *)_ar,NUMSAMPLES);
}
double Measurable::average(int num){
  return _stats->average((float *)_ar,num);
}
double Measurable::cv(){
  return _stats->CV((float *)_ar,NUMSAMPLES);
}
double Measurable::cv(int num){
  return _stats->CV((float *)_ar,num);
}
void Measurable::newVal(double val){
  if(_lastRead-millis()> _interval){
    _newVal(val);
    _lastRead=millis();
  }
}
void Measurable::_newVal(double val){
    _ar[_counter]=val;
    _counter++;
    if(_counter==NUMSAMPLES)
      _counter=0;
}
void Measurable::reset(){
  for(int i = 0 ; i < NUMSAMPLES; i ++)
    _newVal(0.0);

   _counter=0;
}


