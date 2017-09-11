#include "include.h"
#include "pinout.h"

#define DEBUG 2

//piatto vuoto 18.5 gradi
//2.32 minuti di prima oscillazione
Fridge fridge(FRIDGE);
Plate plate(PLATE, 255);
//Plate ring(RING, 180);

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
//TempSensor tRing(TEMP_RING, BCOEFF, 99900, 100000, 25);


//PID control

//Define Variables we'll be connecting to
double //Setpoint, è il valore del wrapper button
//tempPlate, //aggiornato tramite tPlate.getTemp();
outputPlate; //

//Specify the links and initial tuning parameters
double KpPlate=2, KiPlate=1.2, KdPlate=0.0;

AnalogPid platePid(&tPlate.value, & outputPlate,& (buttonPlate.dValue) , KpPlate, KiPlate,KdPlate);

Fridge::STATE outputAmb;

double KpAmb=0.5, KiAmb=0.45;
double threshold = 5.0;
long interval = 20000;

DiscretePid fridgePid(&tAmb.value,&outputAmb, & (buttonAmb.dValue),KpAmb,KiAmb,interval,threshold);

//Statistics control

double _ar1[40];
Measurable statics1(500,_ar1,40);

const int NUMOFRCTMESURE=75;
double _arRct[NUMOFRCTMESURE];
Measurable cycleRCT(0,_arRct,NUMOFRCTMESURE);
int counterRCT=-20;

double _ar2[20];
Measurable totalRCT(0,_ar2,20);
const long NUMMESUREMENT=20;
int counterRCTCycle=0;

//variabili del supervisor

enum STATEMODE { NORMAL, RCTIDLE, RCTMESURE, RCTINSERT, RCTINSERT1, RCTINSERT2, RCTINSERT3, RCTINSERTEND };
STATEMODE stateMode = STATEMODE::NORMAL;
int cifra;
double tempRCT;

enum STATE { IDLE, COOLING, HEATING, MESUREMENT, END ,SELECTIONMODE};
STATE stato= STATE::IDLE;

const int intervalCallDisplay=1000,intervalDisplay=5000;
long lastDisplay=0,displayTimer1=0,displayTimer2=0;
long timeOfLoop;//,lastDisplay=0;;

long lastTimeReadStableState=0, intervalReadStableState=3500; 
unsigned int counterStableState=0;
const unsigned int NUMBEROFSTABLESTATE = 40;

void setup() {
 setLimit();
  
  lcd.init();                        
  lcd.backlight();      
  lcd.clear();
  #ifdef DEBUG
  Serial.begin(2000000);
  #endif
  pinMode(FAN,OUTPUT);
  analogWrite(FAN,255);
  delay(200);
  analogWrite(FAN,50);

  if(btnPUP.getState()==Button::STATE::DOWN && btnPDOWN.getState()==Button::STATE::DOWN){
    stato = STATE::SELECTIONMODE;
  }
}

void loop() {
  timeOfLoop=millis();
  /*supervisor.*/
  _update();// aggiorna ed esegue calcoli sui valori aggiornati
  /*supervisor.*/
  _run();//esegua le operazioni di modifica dello stato quale accensione o spegnimento degli elementi
  
  if(millis()- lastDisplay >= intervalCallDisplay){
    _display();//mostra a display i valori
    lastDisplay=millis();
/*
 lcd.setCursor(0,0);           lcd.print(F("Pmean: "));          lcd.print(statics1.median());
          lcd.setCursor(11,0);          lcd.print(F("Pcv: "));          lcd.print(statics1.cv());
          lcd.setCursor (0,1);          lcd.print(F("Tmean:  "));          lcd.print(statics2.median());
          lcd.setCursor(11,1);          lcd.print(F("Tcv:"));          lcd.print(statics2.cv());
 */
    #ifdef DEBUG
      Serial.print(F("OutAmb: "));
      Serial.print(outputPlate);
      Serial.print(F(" outPidAmb: "));
      Serial.print(fridgePid._output);
      Serial.print(F(" threshold: "));
      Serial.print(fridgePid._threshold);
      Serial.print(F(" | OutPlate: "));
      Serial.print(outputPlate);/*
      Serial.print(" outputMedian: ");
      Serial.print(statics1.median());
      Serial.print(" outputCv: ");
      Serial.print(statics1.cv());*/
      Serial.print(F(" dTime: "));
      Serial.println(millis() - timeOfLoop);
      Serial.print(F("TempPlate: "));
      Serial.print(tPlate.value);
      Serial.print(F(" SetPointPlate: "));
      Serial.println(buttonPlate.value);
      Serial.print(F("TempAmb: "));
      Serial.print(tAmb.value);
      Serial.print(F(" SetPointPlate: "));
      Serial.println(buttonAmb.value);
      Serial.print(F("Stato corrente: "));
      Serial.println(stato);
    #endif
  }

}

//var glob stato
void _run(){
  switch(stato){
    case STATE::IDLE:
      fridge.turn(Fridge::STATE::OFF);
    //  plate.turn(Plate::STATE::OFF);
      plate.set(0);
      break;
    case STATE::COOLING:
     // plate.turn(Plate::STATE::OFF);
      fridge.turn(outputAmb);
      analogWrite(FAN,255);
      break;
    case STATE::HEATING:
     // plate.turn(Plate::STATE::ON);
      analogWrite(FAN,50);
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      break;
    case STATE::MESUREMENT:
      fridge.turn(outputAmb);
      plate.set(outputPlate);
      break;
    case STATE::END:
      plate.turn(Plate::STATE::OFF);
      fridge.turn(outputAmb);
      plate.set(0);
      break;
    case STATE::SELECTIONMODE:
      fridge.turn(Fridge::STATE::OFF);
      plate.set(0);
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
        stato=STATE::COOLING;
        displayTimer1=0;
        displayTimer2=0;
        lcd.clear();
       
      }
      
      break;
     case STATE::COOLING:
     
      tAmb.checkChange();
      tPlate.checkChange();
      fridgePid.compute();
        if(buttonAmb.value  + 0.3 >= tAmb.value ){
          stato=STATE::HEATING;
          displayTimer1=0;
          displayTimer2=0;
          break;
        }
      break;
      
    case STATE::HEATING:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();
     if(millis()-lastTimeReadStableState>= intervalReadStableState){
      if(buttonAmb.value - 1.8 <= tAmb.value && buttonAmb.value + 0.8 >= tAmb.value && buttonPlate.value - 0.8 <= tPlate.value && buttonPlate.value + 1.0 >= tPlate.value){
        counterStableState++;
      }
       else{
        counterStableState=0;
       }
        lastTimeReadStableState= millis();
      }
      //counter time is : NUMBEROFSTABLESTATE * itnervalReadStableState
      if(counterStableState==NUMBEROFSTABLESTATE){
     // if(counterStableState==3){
          stato=STATE::MESUREMENT;
          displayTimer1=0;
          displayTimer2=0;
          lcd.clear();
          //todo azzerare statics1 e 2
          statics1.reset();
      }
    break;
    
    case STATE::MESUREMENT:

    if(btnNEXT.getState()==Button::STATE::DOWN){
        stato=STATE::END;
        displayTimer1=0;
        displayTimer2=0;
        lcd.clear();
      }
    
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();

      statics1.newVal(outputPlate);

      if(counterRCT>=NUMOFRCTMESURE){
          if(counterRCTCycle>=NUMMESUREMENT){
            stato=STATE::END;
            displayTimer1=0;
            displayTimer2=0;
            lcd.clear();
          }else{
            totalRCT.newVal(cycleRCT.median(counterRCT));
            counterRCT=0;
            counterRCTCycle++;
          }
      }

     if(millis()-lastTimeReadStableState>= intervalReadStableState){
      if(counterRCT<=-1){
       counterRCT++;
      }else {
        double tempRCT = convertToRCT(statics1.median(counterRCT),tPlate.value,tAmb.value);
        /*if(tempRCT<=0.010||tempRCT >= 10)
          break;*/
        counterRCT++;
        cycleRCT.newVal( normalize(tempRCT));
      }
      
      lastTimeReadStableState=millis();
     }

     
      break;

    case STATE::END:
      tAmb.checkChange();
      tPlate.checkChange();
      fridgePid.compute();
      plate.turn(Plate::STATE::OFF);
      if(btnNEXT.getState()==Button::STATE::DOWN){
        stato=STATE::IDLE;
        displayTimer1=0;
        displayTimer2=0;
        lcd.clear();
        counterRCT=-30;
      }
      break;

    case STATE::SELECTIONMODE://inseriamo un rct er 20,10,0 e -10 
      tAmb.checkChange();
      tPlate.checkChange(); //NORMAL, RCTIDLE, RCTAMBTEMPSELECTION, RCTMESURE

      //cambiare la modalità
      if(btnPUP.getState() == Button::STATE::DOWN || btnPDOWN.getState() == Button::STATE::DOWN){
        switch(stateMode){
          case STATEMODE::NORMAL: stateMode = STATEMODE::RCTMESURE; break;
          case STATEMODE::RCTIDLE: stateMode = STATEMODE::RCTINSERT; break;
          case STATEMODE::RCTINSERT: stateMode = STATEMODE::NORMAL; break;
        }
      }
      
      if(stateMode==STATEMODE::RCTMESURE){
         if(btnAUP.getState() == Button::STATE::DOWN || btnADOWN.getState() == Button::STATE::DOWN){
            
            if(buttonAmb.value -10 <= -20)
               buttonAmb.set(20);
            else
               buttonAmb.set(buttonAmb.value -10);
         }
      }else if (stateMode==STATEMODE::RCTINSERT){
        if(btnAUP.getState() == Button::STATE::DOWN || btnADOWN.getState() == Button::STATE::DOWN){
            
            if(buttonAmb.value -10 <= -20)
               buttonAmb.set(20);
            else
               buttonAmb.set(buttonAmb.value -10);
         }
      }
      else if(stateMode==STATEMODE::RCTINSERT1 || stateMode==STATEMODE::RCTINSERT2 || stateMode==STATEMODE::RCTINSERT3 ){
        if(btnPUP.getState() == Button::STATE::DOWN){
          cifra++;
          }else if (btnPDOWN.getState() == Button::STATE::DOWN){
           cifra--;
        }  
      }
      
      if(btnNEXT.getState()==Button::STATE::DOWN){
        switch(stateMode){
          case STATEMODE::NORMAL: stato = STATE::IDLE; break;
          case STATEMODE::RCTIDLE: stateMode = STATEMODE::RCTINSERT; break;
          case STATEMODE::RCTMESURE: stato= STATE::COOLING; break;
          case STATEMODE::RCTINSERT: stateMode = STATEMODE::RCTINSERT;tempRCT= 0.; cifra=5; break;
          case STATEMODE::RCTINSERT1: tempRCT += 0.1 * cifra; cifra = 5; stateMode = STATEMODE::RCTINSERT2; break;
          case STATEMODE::RCTINSERT2: tempRCT += 0.01 * cifra; cifra = 5; stateMode = STATEMODE::RCTINSERT3; break;
          case STATEMODE::RCTINSERT3: tempRCT += 0.001 * cifra; cifra = 5; stateMode = STATEMODE::RCTINSERTEND; break;
          case STATEMODE::RCTINSERTEND: stato = STATEMODE::RCTINSERTEND; break;
        }
          displayTimer1=0;
          displayTimer2=0;
          lcd.clear();
      }
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
        //lcd.setCursor (0,2);         lcd.print(F("T da raggiungere:  "));  
        //lcd.setCursor(0,3); lcd.print(calculateCollingValue(buttonAmb.dValue),1);
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
      }else if(displayTimer1==1) {
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
          lcd.setCursor(0,0);           lcd.print(F("OutP: "));          lcd.print(outputPlate);
          //lcd.setCursor(11,0);          lcd.print(F("Pcv: "));          lcd.print(statics1.cv());
          lcd.setCursor (0,1);          lcd.print(F("OutA: "));          lcd.print(outputAmb);
          //lcd.setCursor(11,1);          lcd.print(F("Tcv:"));          lcd.print(statics2.cv());
          lcd.setCursor(0,2);          lcd.print(F("countSS:"));          lcd.print(counterStableState);
          lcd.setCursor(0,3); lcd.print(F("Goal: ")); lcd.print(NUMBEROFSTABLESTATE);
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            displayTimer2=millis();
            lcd.clear();
           }
        }
      
      break;
    case STATE::MESUREMENT://TODO
      if(displayTimer1==0){//diplayTimer1 mostra a che fase è del singolo stato mentre displayTimer2 indica quale informazione stampare a video
        lcd.setCursor(0,0);        lcd.print(F("Stato: MISURA"));       
       lcd.setCursor(0,1);        lcd.print(F("Campion. in corso."));
       lcd.setCursor(0,2) ;        lcd.print(F("Premere next per fermare le misurazioni"));
        if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=1;
            lcd.clear();
         }
      }else if(displayTimer1==1) {
            if(counterRCT >= 0 ){
              double temp=cycleRCT.median(counterRCT);
              lcd.setCursor(0,0);         lcd.print(F("RCTinst: "));        lcd.print(temp,4);
              lcd.setCursor(0,1);        lcd.print(F("CLOinst: "));        lcd.print(rct2Clo(temp),4);
              temp=totalRCT.average(counterRCTCycle);
              lcd.setCursor(0,2);         lcd.print(F("RCTm: "));        lcd.print(temp,4);
              lcd.setCursor(0,3);        lcd.print(F("CLOm: "));        lcd.print(rct2Clo(temp),4);
               if(millis() - displayTimer2 >= intervalDisplay){
                displayTimer1=2;
                displayTimer2=millis();
                lcd.clear();
               }
            }else{
              lcd.setCursor(0,2);         lcd.print(F("Campion. di stabilz."));
              lcd.setCursor(0,3);         lcd.print(F("in corso "));
            }
      }else if (displayTimer1==2){
        lcd.setCursor(0,0);         lcd.print(F("Ta: "));        lcd.print(tAmb.value,1);   lcd.print(F(" Tc: "));        lcd.print(tPlate.value,1);
        lcd.setCursor(0,2); lcd.print(F("N inst: ")); lcd.print(counterRCT);
        lcd.setCursor(0,3); lcd.print(F("N media: ")); lcd.print(counterRCTCycle);
        
        if(millis() - displayTimer2 >= intervalDisplay){
          displayTimer1=1;
          displayTimer2=millis();
          lcd.clear();
         }
       }
      
      break;

    case STATE::END:
          
      if(displayTimer1==0){//diplayTimer1 mostra a che fase è del singolo stato mentre displayTimer2 indica quale informazione stampare a video
         lcd.setCursor(0,0);          lcd.print(F("Misura conclusa."));
         lcd.setCursor(0,1);          lcd.print(F("Premi NEXT per"));
         lcd.setCursor(0,2);          lcd.print(F("riavviare la macchina"));
         if(millis() - displayTimer2 >= intervalDisplay + 10000){
            displayTimer1=1;
            lcd.clear();
         }
      }else{
        lcd.setCursor(0,0);        lcd.print(F("Ta: "));        lcd.print(buttonAmb.value,1);     lcd.print(F(" Tp:"));        lcd.print(buttonPlate.dValue,1);
        lcd.setCursor (0,1);        lcd.print(F("RCT medio:  "));        lcd.print(totalRCT.average(counterRCTCycle),3);
        lcd.setCursor(0,2);        lcd.print(F("Clo medio:"));        lcd.print(rct2Clo(totalRCT.average(counterRCTCycle)),3);
        lcd.setCursor(0,3); lcd.print(F("Calc. su ")); lcd.print(counterRCTCycle);
     }
      break;
     case STATE::SELECTIONMODE:

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
  plate.turn(Plate::STATE::OFF);
//  ring.turn(Plate::STATE::OFF);
  while(1){
    Serial.print(".");
  }
}
void setLimit(){
  tAmb.setLimit(30,900,callback);
  tPlate.setLimit(30,900,callback);
//  tRing.setLimit(30,900,callback);
  
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
/*
float calculateCollingValue(float val){
  return 1.0714*val - 2.42857;
}*/
