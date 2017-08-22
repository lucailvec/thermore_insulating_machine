
#include "DiscretePid.h"

#ifndef PID_V1_H
#include <PID_v1.h>
#endif

#include "Fridge.h"

DiscretePid::DiscretePid(double * setpoint, Fridge::STATE * output,double * mesure,double kp,double ki,long interval, double threshold){
  _outputDiscrete = output;
  _threshold = threshold;
  _interval = interval;
  pid = new PID( setpoint, &_output, mesure, kp, ki, 0., P_ON_M, DIRECT);
  pid->SetMode(AUTOMATIC);
}

void DiscretePid::compute(){
  pid->Compute();
  if(millis() - _lastOn>=_interval && ( _output >= _threshold || _output <= - _threshold ) ) {//da controllare perchè non so come venga gestito il reverse
    *_outputDiscrete = ( _output >0) ?Fridge::STATE::ON : Fridge::STATE::OFF; //a causa del DIRECT
    _lastOn = millis();
  }
}

