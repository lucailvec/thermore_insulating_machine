#include "include.h"
#include "pinout.h"

#define DEBUG 2

Fridge fridge(FRIDGE);
Plate plate(PLATE, 255);
Plate ring(RING, 125);

Button btnPUP(BTN_TEMP_PLATE_UP, 500);
Button btnPDOWN(BTN_TEMP_PLATE_DOWN, 500);
Button btnAUP(BTN_TEMP_AMB_UP, 500);
Button btnADOWN(BTN_TEMP_AMB_DOWN, 500);
Button btnNEXT(BTN_NEXT, 500);

WrapperButton buttonPlate(35, &btnPUP, &btnPDOWN,minTempPlate,maxTempPlate);
WrapperButton buttonAmb(20, &btnAUP, &btnADOWN,minTempAmb,maxTempAmb);

PCF8574_HD44780_I2C lcd(0x27,20,4);

#define RNOMINALE 100000
#define TNOMINALE 25
#define BCOEFF 3955

// TempSensor(int numPin, int B, int R, int R0, float T0);
TempSensor tAmb(TEMP_AMB, BCOEFF, 297000, 100000, 25);
TempSensor tPlate(TEMP_PLATE, BCOEFF, 99800, 100000, 25);
TempSensor tRing(TEMP_RING, BCOEFF, 99900, 100000, 25);


//PID control

//Define Variables we'll be connecting to
double //Setpoint, è il valore del wrapper button
//tempPlate, //aggiornato tramite tPlate.getTemp();
outputPlate; //

//Specify the links and initial tuning parameters
double KpPlate=2, KiPlate=0.5;

AnalogPid platePid(&tPlate.value, & outputPlate,& (buttonPlate.dValue) , KpPlate, KiPlate);
/*
double mesureP,setpointP;
PID * platePid = new PID(&mesureP, & (outputPlate),& setpointP , KpPlate, KiPlate, 0.0 , DIRECT);

double Setpoint=2, Input=1, Output;

//Specify the links and initial tuning parameters
double Kp=2, Ki=5, Kd=1;
PID * myPid = new PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
*/
Fridge::STATE outputAmb;

double KpAmb=1, KiAmb=0.5, KdAmb=0.01;
double threshold = 5.0;
long interval = 5000;
//(double *setpoint, Fridge::STATE * output,double *mesure,double kp,double ki,long interval, double threshold);
DiscretePid fridgePid(&tAmb.value,&outputAmb, & (buttonAmb.dValue),KpAmb,KiAmb,interval,threshold);
//Statistics control

double _ar1[40];
double _ar2[40];
Measurable statics1(200,_ar1,40);
Measurable statics2(200,_ar2,40);
long _numMesurement=40;

//variabili del supervisor

enum STATE { IDLE, HEATING, MESUREMENT };
STATE stato;
long intervalCallDisplay=1000,intervalDisplay=5000,lastDisplay=0,displayTimer1=0,displayTimer2=0;
long timeOfLoop;//,lastDisplay=0;;

//pid piatto


void callback(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(F("ATTENZIONE, controllare sensori"));
  Serial.println("ATTENZIONE errore, chiamata callback");
  fridge.turn(Fridge::STATE::OFF);
  plate.set(0);
  ring.set(0);
  while(1){
    Serial.print(".");
  }
}
void setLimit(){
  tAmb.setLimit(30,900,callback);
  tPlate.setLimit(30,900,callback);
  tRing.setLimit(30,900,callback);
  
}
void setup() {
  setLimit();
  
  lcd.init();                        
  lcd.backlight();      
  lcd.clear();
  Serial.begin(2000000);


  
 // myPid->SetMode(AUTOMATIC);
}

void loop() {
  timeOfLoop=millis();
  /*supervisor.*/
  _run();//esegua le operazioni di modifica dello stato quale accensione o spegnimento degli elementi
  /*supervisor.*/
  _update();// aggiorna ed esegue calcoli sui valori aggiornati

  if(millis()- lastDisplay >= intervalCallDisplay){
    _display();//mostra a display i valori
    lastDisplay=millis();
  }
  #ifdef DEBUG
  Serial.println(millis() - timeOfLoop);
  #endif
}

//var glob stato
void _run(){
  switch(stato){
    case STATE::IDLE:
      fridge.turn(Fridge::STATE::OFF);
      plate.set(0);
      ring.set(0);
      break;
    case STATE::HEATING:
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      break;
    case STATE::MESUREMENT:
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      break;
  }
}

void _update(){
 /*Input= tPlate.value;
  Setpoint=buttonPlate.value;
*/
  
  switch(stato){
    case STATE::IDLE:
      tAmb.checkChange();
      tPlate.checkChange();
      buttonPlate.checkChange();
      buttonAmb.checkChange();
      
      if(btnNEXT.getState()==Button::STATE::DOWN){
        stato=STATE::HEATING;
        displayTimer1=0;
        displayTimer2=0;
        lcd.clear();
      }
      
      break;
    case STATE::HEATING:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      //myPid->Compute();
      Serial.print(F("Compute analogPid with input of:: mesure: "));
      Serial.print(tPlate.value);
      Serial.print(" setpoint :");
      Serial.print(buttonPlate.value);
      Serial.print(" and give an output of: ");
      Serial.println(outputPlate);
      
      fridgePid.compute();

      Serial.print(F("Compute discrete with input of:: mesure: "));
      Serial.print(tAmb.value);
      Serial.print(" setpoint :");
      Serial.print(buttonAmb.dValue);
      Serial.print(" and give an output of: ");
      Serial.println(outputAmb);
      
      fridge.turn(outputAmb);
      plate.set(outputPlate);

      
      statics1.newVal(outputPlate);
      statics2.newVal(tPlate.value);

      if(false){//guardo se la deviazione standard rispetto alla media scende sotto il 10 % se non di meno
        stato=STATE::MESUREMENT;
        displayTimer1=0;
        displayTimer2=0;
      }
      
    case STATE::MESUREMENT:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      statics1.newVal(outputAmb);
      
      break;
  }
}

//long intervalDisplay=3000,lastDisplay=0,displayTimer1=0,displayTimer2=0,_numMesurement=30;

//var glob stato
void _display(){
  if(displayTimer1==0 && displayTimer2==0)
   displayTimer2=millis();
  switch(stato){
    case STATE::IDLE:
        
      if(displayTimer1==0){//diplayTimer1 mostra a che fase è del singolo stato mentre displayTimer2 indica quale informazione stampare a video
         lcd.setCursor(0,0); //prima riga
         lcd.print(F("Stato: IDLE"));
         lcd.setCursor(0,2); //prima riga
         lcd.print(F("POI premi  NEXT"));
         lcd.setCursor(0,1); //prima riga
         lcd.print(F("Sett. la temp"));
         if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            lcd.clear();
         }
      }else{
        lcd.setCursor(0,0); //prima riga
        lcd.print(F("Ta: "));
        lcd.print(tAmb.value,1);
        lcd.setCursor(12,0);
        lcd.print(F("Tap:"));
        lcd.print(buttonAmb.value);
        lcd.setCursor (0,1);//seconda riga 
        lcd.print(F("Tc:  "));
        lcd.print(tPlate.value,1);
        lcd.setCursor(12,1);
        lcd.print(F("Tcp:"));
        lcd.print(buttonPlate.value);
     }
      
      break;
    case STATE::HEATING:
      if(displayTimer1==0){
       lcd.setCursor(0,0); //prima riga
       lcd.print(F("Stato: HEATING"));
       lcd.setCursor(0,1); //prima riga
       lcd.print(F("Fase di stabilizzazione"));
       lcd.setCursor(0,2); //prima riga
       lcd.print(F("Attendere prego."));
       if(millis() - displayTimer2 >= intervalDisplay){
        displayTimer1=1;
        displayTimer2=millis();
        lcd.clear();
       }
      }else{
        if(displayTimer1==1) {
          lcd.setCursor(0,0); //prima riga
          lcd.print(F("Ta: "));
          lcd.print(tAmb.value,1);
          lcd.setCursor(12,0);
          lcd.print(F("Tap:"));
          lcd.print(buttonAmb.value);
          lcd.setCursor (0,1);//seconda riga 
          lcd.print(F("Tc:  "));
          lcd.print(tPlate.value,1);
          lcd.setCursor(12,1);
          lcd.print(F("Tcp:"));
          lcd.print(buttonPlate.value);
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=2;
            displayTimer2=millis();
            lcd.clear();
           }
        }else if (displayTimer1==2){
          lcd.setCursor(0,0); //prima riga
          lcd.print(F("Pmean: "));
          lcd.print(statics1.median());
          lcd.setCursor(0,1);
          lcd.print(F("CV: "));
          lcd.print(statics1.cv());
          lcd.setCursor (0,2);//seconda riga 
          lcd.print(F("Tmean:  "));
          lcd.print(statics2.median());
          lcd.setCursor(0,3);
          lcd.print(F("Tsd:"));
          lcd.print(statics2.cv());
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            displayTimer2=millis();
            lcd.clear();
           }
        }
      }
      break;
    case STATE::MESUREMENT://TODO
      break;
  }
  
 
}
  


