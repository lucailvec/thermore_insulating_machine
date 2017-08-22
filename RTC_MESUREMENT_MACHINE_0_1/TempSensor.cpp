
#include "TempSensor.h"

const float ZERODEGREEKELVIN = 273.15;

TempSensor::TempSensor(int numPin, int BCOEFFICIENT, int SERIESRESISTOR, int THERMISTORNOMINAL, float TEMPERATURENOMINAL){
  _BCOEFFICIENT = BCOEFFICIENT;
  _THERMISTORNOMINAL=THERMISTORNOMINAL;
  _TEMPERATURENOMINAL=TEMPERATURENOMINAL;
  _SERIESRESISTOR=SERIESRESISTOR;
  _numPin=numPin;  

  _minADC = 0;
  _maxADC = 1023;

  for(int i = 0 ; i < 5 ;  i++)
    TempSensor::_readNewValue();
  
}

void TempSensor::_readNewValue(){
  _ar[_counter]=_getTemp();
  _counter++;
  if(_counter == 5)
    _counter = 0;
}
void TempSensor::setLimit(int minADC, int maxADC, void (*callback)()){
  _minADC = minADC;
  _maxADC = maxADC;
  _callback =callback;
}
/*
void TempSensor::_noCallBCOEFFICIENTack(){
  }*/
bool TempSensor::isResistanceConnected(int val){
  if(val<this->_minADC || val > this->_maxADC ) {
    _callback();
    return false;
  }
  return true;
}

float TempSensor::_getTemp(){
 int res = analogRead(_numPin);
 float average;
 
 if( ! isResistanceConnected(res) )
  return 666.0;

  average = 1023.0 / res - 1;
  average = _SERIESRESISTOR / average;
 
  float steinhart;
  steinhart = average / _THERMISTORNOMINAL;     // (R/Ro)
  steinhart = log(steinhart);                  // ln(R/Ro)
  steinhart /= _BCOEFFICIENT;                   // 1/B * ln(R/Ro)
  steinhart += 1.0 / (_TEMPERATURENOMINAL + ZERODEGREEKELVIN); // + (1/To)
  steinhart = 1.0 / steinhart;                 // Invert
  steinhart -= ZERODEGREEKELVIN;                         // convert to C

  return steinhart;
}

void TempSensor::checkChange(){
  _readNewValue();
  float average;
  for(int i = 0 ; i < 5 ; i++)
    average+=this->_ar[i];
  this->value= (double)average/5;
}

