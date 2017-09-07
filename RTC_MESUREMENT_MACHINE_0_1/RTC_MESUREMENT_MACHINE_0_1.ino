#include "include.h"
#include "pinout.h"

#define DEBUG 2

Fridge fridge(FRIDGE);
Plate plate(PLATE, 200);
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

TempSensor tAmb(TEMP_AMB, BCOEFF, 297000, 100000, 25);
TempSensor tPlate(TEMP_PLATE, BCOEFF, 99800, 100000, 25);
TempSensor tRing(TEMP_RING, BCOEFF, 99900, 100000, 25);


//PID control

//Define Variables we'll be connecting to
double //Setpoint, è il valore del wrapper button
//tempPlate, //aggiornato tramite tPlate.getTemp();
outputPlate; //

//Specify the links and initial tuning parameters
double KpPlate=2, KiPlate=0.01;

AnalogPid platePid(&tPlate.value, & outputPlate,& (buttonPlate.dValue) , KpPlate, KiPlate);

Fridge::STATE outputAmb;

double KpAmb=1, KiAmb=0.75, KdAmb=0.03;
double threshold = 5.0;
long interval = 20000;

DiscretePid fridgePid(&tAmb.value,&outputAmb, & (buttonAmb.dValue),KpAmb,KiAmb,interval,threshold);

//Statistics control

double _ar1[40];
Measurable statics1(500,_ar1,40);

double _ar2[40];
Measurable statics2(500,_ar2,40);
long _numMesurement=40;

double _arRct[40];
Measurable staticsRCT(1000,_arRct,40);
int counterRCT=0;


//variabili del supervisor

enum STATE { IDLE, COOLING, HEATING, MESUREMENT, END };
STATE stato;
long intervalCallDisplay=1000,intervalDisplay=5000,lastDisplay=0,displayTimer1=0,displayTimer2=0;
long timeOfLoop;//,lastDisplay=0;;

bool stableState= false; unsigned int counterStableState=0;

void setup() {
  setLimit();
  
  lcd.init();                        
  lcd.backlight();      
  lcd.clear();
  Serial.begin(2000000);

  pinMode(FAN,OUTPUT);
  digitalWrite(FAN,255);
  delay(1000);
  digitalWrite(FAN,30);
  
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
    case STATE::COOLING:
      fridge.turn(outputAmb);
      fan.set(255);
      break;
    case STATE::HEATING:
      fan.set(30);
      fridge.turn(outputAmb);
      if(stableState)
        plate.set(outputPlate);
      break;
    case STATE::MESUREMENT:
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      break;
    case STATE::END:
      fridge.turn(outputAmb);
      plate.set(0);
      ring.set(0);
      break;
  }
}

void _update(){
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
     case STATE::COOLING:
      tAmb.checkChange();
      tPlate.checkChange();
      fridgePid.compute();

    if(calculateCollingValue(buttonAmb.value)  >= tAmb.value ){//&& buttonPlate.value - 0.8 <= statics2.median() && buttonPlate.value + 0.8 >= statics2.median()){
      stato=STATE::HEATING;
      displayTimer1=0;
      displayTimer2=0;
    }

      break;
    case STATE::HEATING:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();

  
     if(buttonAmb.value - 0.4 <= statics1.median() && buttonAmb.value + 0.4 >= statics1.median() ){//&& buttonPlate.value - 0.8 <= statics2.median() && buttonPlate.value + 0.8 >= statics2.median()){

      counterStableState++;
      
      if(counterStableState==3){
        stableState=true;
       
      }else if(counterStableState==10){
         if(buttonAmb.value - 0.4 <= statics1.median() && buttonAmb.value + 0.4 >= statics1.median() && buttonPlate.value - 0.8 <= statics2.median() && buttonPlate.value + 0.8 >= statics2.median()){
          stato=STATE::MESUREMENT;
          displayTimer1=0;
          displayTimer2=0;
        }
      }
     }else{
      counterStableState=0;
     }

     break;
    //devo temporizzare le misure, leggo una misura ogni secondo ? -> definisco una costante chiamata intervallo measurament.
    case STATE::MESUREMENT:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();
      
      statics1.newVal(outputPlate);
      statics2.newVal(tPlate.value);
      
      counterRCT++;
      staticsRCT.newVal( normalize(convertToRCT(statics1.median(counterRCT))) ,statics2.median(),tAmb.value));
      
      break;

    case STATE::END:
      tAmb.checkChange();
      tPlate.checkChange();
      fridgePid.compute();
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
         lcd.setCursor(0,0);          lcd.print(F("Stato: IDLE"));
         lcd.setCursor(0,2);          lcd.print(F("POI premi  NEXT"));
         lcd.setCursor(0,1);          lcd.print(F("Sett. la temp"));
         if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            lcd.clear();
         }
      }else{
        lcd.setCursor(0,0);        lcd.print(F("Ta: "));        lcd.print(tAmb.value,1);
        lcd.setCursor(12,0);        lcd.print(F("Tap:"));        lcd.print(buttonAmb.dValue,1);
        lcd.setCursor (0,1);        lcd.print(F("Tc:  "));        lcd.print(tPlate.value,1);
        lcd.setCursor(12,1);        lcd.print(F("Tcp:"));        lcd.print(buttonPlate.value);
     }
      
      break;
    case STATE::COOLING:
      if(displayTimer1==0){
       lcd.setCursor(0,0);        lcd.print(F("Stato: COOLING"));       
       lcd.setCursor(0,2) ;        lcd.print(F("Fase di raffreddamento"));
       lcd.setCursor(0,1);        lcd.print(F("Attendere prego."));
       if(millis() - displayTimer2 >= intervalDisplay){
        displayTimer1=1;
        lcd.clear();
       }
      }else{
        lcd.setCursor(0,0);          lcd.print(F("Ta: "));          lcd.print(tAmb.value,1);
        lcd.setCursor(0,1);          lcd.print(F("Tap:"));          lcd.print(buttonAmb.dValue,1);
        lcd.setCursor (0,2);         lcd.print(F("T da raggiungere:  "));  
        lcd.setCursor(0,3); lcd.print(tPlate.value,1);
      }
    break;
    case STATE::HEATING:
      if(displayTimer1==0){
       lcd.setCursor(0,0);        lcd.print(F("Stato: HEATING"));       
       lcd.setCursor(0,2) ;        lcd.print(F("Fase di stabilizzazione"));
       lcd.setCursor(0,1);        lcd.print(F("Attendere prego."));
       if(millis() - displayTimer2 >= intervalDisplay){
        displayTimer1=1;
        displayTimer2=millis();
        lcd.clear();
       }
      }else{
        if(displayTimer1==1) {
          lcd.setCursor(0,0);          lcd.print(F("Ta: "));          lcd.print(tAmb.value,1);
          lcd.setCursor(12,0);          lcd.print(F("Tap:"));          lcd.print(buttonAmb.dValue,1);
          lcd.setCursor (0,1);         lcd.print(F("Tc:  "));          lcd.print(tPlate.value,1);
          lcd.setCursor(12,1);          lcd.print(F("Tcp:"));          lcd.print(buttonPlate.value);
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=2;
            displayTimer2=millis();
            lcd.clear();
           }
        }else if (displayTimer1==2){
          lcd.setCursor(0,0);           lcd.print(F("Pmean: "));          lcd.print(statics1.median());
          lcd.setCursor(11,0);          lcd.print(F("CV: "));          lcd.print(statics1.cv());
          lcd.setCursor (0,1);          lcd.print(F("Tmean:  "));          lcd.print(statics2.median());
          lcd.setCursor(11,1);          lcd.print(F("Tsd:"));          lcd.print(statics2.cv());
          lcd.setCursor(0,2);          lcd.print(F("countSS:"));          lcd.print(counterStableState);
          lcd.setCursor(0,3); lcd.print(F("p ON?: ")); lcd.print(stableState);
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            displayTimer2=millis();
            lcd.clear();
           }
        }
      }
      break;
    case STATE::MESUREMENT://TODO
        lcd.setCursor(0,0);         lcd.print(F("Ta: "));        lcd.print(tAmb.value,1);
        lcd.setCursor(12,0);        lcd.print(F("Tap:"));        lcd.print(buttonAmb.dValue,1);
        lcd.setCursor (0,1);        lcd.print(F("Tc:  "));        lcd.print(tPlate.value,1);
        lcd.setCursor(12,1);        lcd.print(F("Tcp:"));        lcd.print(buttonPlate.value);
        double tempRCT= staticsRCT.median(counterRCT); 
        lcd.setCursor(0,2);         lcd.print(F("RCT: "));        lcd.print(tempRCT,2);
        lcd.setCursor(0,3);        lcd.print(F("CLO: "));        lcd.print(tempRCT*6.45,2);
        lcd.setCursor(11,3); lcd.print(F("N°: ")); lcd.print(counterRCT);
      break;

    case STATE::END:
          
      if(displayTimer1==0){//diplayTimer1 mostra a che fase è del singolo stato mentre displayTimer2 indica quale informazione stampare a video
         lcd.setCursor(0,0);          lcd.print(F("Misura conclusa."));
         lcd.setCursor(0,1);          lcd.print(F("Premi NEXT per"));
         lcd.setCursor(0,1);          lcd.print(F("riavviare la macchina"));
         if(millis() - displayTimer2 >= intervalDisplay + 2000){
            displayTimer1=1;
            lcd.clear();
         }
      }else{
        lcd.setCursor(0,0);        lcd.print(F("Ta: "));        lcd.print(tAmb.value,1);
        lcd.setCursor(12,0);        lcd.print(F("Tap:"));        lcd.print(buttonAmb.dValue,1);
        lcd.setCursor (0,1);        lcd.print(F("RCT:  "));        lcd.print(staticsRCT.median(counterRCT),1);
        lcd.setCursor(12,1);        lcd.print(F("Clo:"));        lcd.print(rct2Clo(buttonPlate.value)));
     }
      break;
  }
  
 
}


double normalize(double val){
  //return asd;
  return val;
}
double convertToRCT(double dutyCyclePlate,double tempPlate,double tempAmb){
  //total area = 0.215*0.215 = 0,046225
  //double res= 1.5; //ohm 12V / 1.5 = 8A
  //W = V * I = 12*8 = 96 Watt
 // double p = 

  return 0.046225 *(tempPlate-tempAmb)/(96. *(dutyCyclePlate/255.))-RCT0;
}

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

/*
 * 
 // a 20 vado in basso di 5 gradi, a -10 vado in basso di 10 gradi
 //   Y = 0.8333*X - 11.67
 
 * 
 */
//Y = 0.8333*X - 11.67
float rct2Clo(float val){
  return val * 6.45;
}
float calculateCollingValue(float val){
  return 0.8333*val - 11.67;
}
