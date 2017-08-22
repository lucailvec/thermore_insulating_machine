#ifndef SUPERVISOR_H
#define SUPERVISOR_H
#include <PID_v1.h>

#ifndef ARDUINO_H
#include "Arduino.h"
#endif

class Supervisor {
  public: Supervisor(TempSensor tAmb,double *amb,double *plate,PCF8574_HD44780_I2C *lcd, WrapperButton *btn);
          enum STATE { IDLE, HEATING, MESUREMENT };
          void run();
          void display();
  private:
          PCF8574_HD44780_I2C *lcd;
          STATE _currentState; 
          
};

#endif
