

#ifndef TempSensor_H
#define TempSensor_H

#ifndef Arduino_H
#include "Arduino.h"
#endif

class TempSensor
{
public: 
  TempSensor(int numPin, int BCOEFFICIENT, int SERIESRESISTOR, int THERMISTORNOMINAL, float TEMPERATURENOMINAL);
  void checkChange();
  double value;
  void setLimit(int minResistance, int maxResistance, void (*callback)());
  bool isResistanceConnected(int val);
private:
  int _BCOEFFICIENT;
  float _THERMISTORNOMINAL;
  float _TEMPERATURENOMINAL;
  float _SERIESRESISTOR;
  int _numPin;
  int _minADC;
  int _maxADC;
  void (*_callback)();
  void __noCallback(void);
//Per gestire la media su 5 valori
  float _getTemp();
  void  _readNewValue();
  float _ar[5];
  unsigned int _counter;
};

#endif
