
#include "DiscretePid.h"

#ifndef PID_V1_H
#include <PID_v1.h>
#endif

#include "Fridge.h"

DiscretePid::DiscretePid(double * mesure, Fridge::STATE * output,double * setpoint,double kp,double ki,long interval, double threshold){
  _outputDiscrete = output;
  _threshold = threshold;
  _interval = interval;
  pid = new PID( mesure, &_output, setpoint, kp, ki, 0., P_ON_M, REVERSE);
  pid->SetMode(AUTOMATIC);
  pid->SetOutputLimits(0,120);
}

void DiscretePid::compute(){
  pid->Compute();
  if(millis() - _lastOn>=_interval ) {//da controllare perchè non so come venga gestito il reverse
    *_outputDiscrete = ( _output >2) ?Fridge::STATE::ON : Fridge::STATE::OFF; //a causa del DIRECT
    _lastOn = millis();
  }
}

