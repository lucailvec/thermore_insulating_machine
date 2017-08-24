
#include "AnalogPid.h"

#ifndef PID_V1_H
#include <PID_v1.h>
#endif


AnalogPid::AnalogPid(double *mesure,double *output,double *setpoint,double kp,double ki){
  /*#ifdef DEBUG
    Serial.print("Inizializzo analogPID con setpoint:");
    Serial.print(*setpoint);
    Serial.print(" output:");
    Serial.print(*output);
    Serial.print("mesure ");
    Serial.println(*mesure);
  #endif*/
  pid = new PID(mesure , output,setpoint , kp, ki, 0.2, DIRECT);
  pid->SetMode(AUTOMATIC);
  
}
void AnalogPid::compute(){
  pid->Compute();
 
}





