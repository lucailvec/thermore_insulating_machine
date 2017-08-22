

#ifndef TEMPSENSOR_H
#define TEMPSENSOR_H
#include <PID_v1.h>

#ifndef Arduino_H
#include "Arduino.h"
#endif

class AnalogPid {
  public: AnalogPid(double *setpoint,double *output,double *mesure,double kp,double ki);
          PID * pid;//tramite l'esposizione del pid espongo tutto quello che mi serve
         void compute();
};

#endif
