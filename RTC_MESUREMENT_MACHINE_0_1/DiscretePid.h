#ifndef DISCRETEPID_H
#define DISCRETEPID_H

#include <PID_v1.h>
#include "Fridge.h"
#ifndef ARDUINO_H
#include "Arduino.h"
#endif

class DiscretePid{
  public: DiscretePid(double *setpoint, Fridge::STATE * output,double *mesure,double kp,double ki,long interval, double threshold);
          PID * pid;
          void compute();
  private: Fridge::STATE * _outputDiscrete;
           double _output;
           long _interval;
           long _lastOn;
           double _threshold;
};

#endif
