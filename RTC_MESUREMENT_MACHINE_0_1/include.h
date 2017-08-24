#include "Fridge.h"
#include "Plate.h"
#include "TempSensor.h"
#include "Button.h"
#include "WrapperButton.h"

#include "Measurable.h"
#include "AnalogPid.h"
#include "DiscretePid.h"

#include <PCF8574_HD44780_I2C.h>
#define ONE_WIRE_BUS 17    

#define minTempPlate 10
#define maxTempPlate 50
#define minTempAmb -20
#define maxTempAmb 40
