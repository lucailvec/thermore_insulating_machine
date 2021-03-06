#ifndef DISCRETEPID_H
#define DISCRETEPID_H

#include <PID_v1.h>
#include "Fridge.h"
#ifndef ARDUINO_H
#include "Arduino.h"
#endif

class DiscretePid{
  public: DiscretePid( double *mesure, Fridge::STATE * output,double *setpoint,double kp,double ki,long interval, double threshold);
          PID * pid;
          void compute();
          double _output;
           double _threshold;
         // void changeSetPoint(double * newSetPoint);
  private: Fridge::STATE * _outputDiscrete;
           long _interval;
           long _lastOn;
           double *_setpoint;
           double *_mesure;
};

#endif
