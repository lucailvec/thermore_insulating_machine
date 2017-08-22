
#include "AnalogPid.h"

#ifndef PID_V1_H
#include <PID_v1.h>
#endif

AnalogPid::AnalogPid(double *setpoint,double *output,double *mesure,double kp,double ki){
  pid = new PID( setpoint, output, mesure, kp, ki, 0.,P_ON_M, DIRECT);
  pid->SetMode(AUTOMATIC);
  
}
void AnalogPid::compute(){
  pid->Compute();
  
}





