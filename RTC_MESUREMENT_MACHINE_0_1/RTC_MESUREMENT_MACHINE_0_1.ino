#include "include.h"
#include "pinout.h"



Fridge fridge(FRIDGE);
Plate plate(PLATE, 125);
Plate ring(RING, 125);

Button btnPUP(BTN_TEMP_PLATE_UP, 200);
Button btnPDOWN(BTN_TEMP_PLATE_DOWN, 200);
Button btnAUP(BTN_TEMP_AMB_UP, 200);
Button btnADOWN(BTN_TEMP_AMB_DOWN, 200);
Button btnNEXT(BTN_NEXT, 200);

WrapperButton buttonPlate(35, &btnPUP, &btnPDOWN);
WrapperButton buttonAmb(20, &btnAUP, &btnADOWN);

PCF8574_HD44780_I2C lcd(0x27,20,4);

#define RNOMINALE 100000
#define TNOMINALE 25
#define BCOEFF 3955

// TempSensor(int numPin, int B, int R, int R0, float T0);
TempSensor tAmb(TEMP_AMB, BCOEFF, 99900, 100000, 25);
TempSensor tPlate(TEMP_PLATE, BCOEFF, 99800, 100000, 25);
TempSensor tRing(TEMP_RING, BCOEFF, 297000, 100000, 25);


//PID control
//Define Variables we'll be connecting to
double //Setpoint, è il valore del wrapper button
//tempPlate, //aggiornato tramite tPlate.getTemp();
outputPlate; //

//Specify the links and initial tuning parameters
double KpPlate=2, KiPlate=5, KdPlate=0;

AnalogPid platePid(& (buttonPlate.value), & outputPlate, &tPlate.value, KpPlate, KiPlate);

Fridge::STATE outputAmb;

double KpAmb=2, KiAmb=5, KdAmb=0;
double threshold = 5.0;
long interval = 5000;
//(double *setpoint, Fridge::STATE * output,double *mesure,double kp,double ki,long interval, double threshold);
DiscretePid fridgePid(& (buttonAmb.value),&outputAmb,&tAmb.value ,KpAmb,KiAmb,interval,threshold);
//Statistics control

Measurable statics1;
Measurable statics2;

//variabili del supervisor

enum STATE { IDLE, HEATING, MESUREMENT };
STATE stato;
long intervalDisplay=3000,lastDisplay=0,displayTimer1=0,displayTimer2=0,_numMesurement=50;
long timeOfLoop;

void callback(){
  Serial.println("ATTENZIONE");
  while(1){
    Serial.println(".");
  }
}
void setLimit(){
  tAmb.setLimit(1,1024,callback);
  tPlate.setLimit(1,1024,callback);
  tRing.setLimit(1,1024,callback);
  
}
void setup() {
  setLimit();
  
  lcd.init();                        
  lcd.backlight();      
  lcd.clear();
  Serial.begin(115200);
}

void loop() {
  timeOfLoop=micros();
  /*supervisor.*/
  _run();//esegua le operazioni di modifica dello stato quale accensione o spegnimento degli elementi
  /*supervisor.*/
  _update();// aggiorna ed esegue calcoli sui valori aggiornati
  _display();//mostra a display i valori
  Serial.println(micros() - timeOfLoop);
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
      }
      
      break;
    case STATE::HEATING:
      tAmb.checkChange();
      tPlate.checkChange();
      platePid.compute();
      fridgePid.compute();
      fridge.turn(outputAmb);
      plate.set(outputPlate);
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
      if(_numMesurement==statics1._numOfValue){
        statics1.reset();
        //statics2.reset();
      }
      statics1.newVal(outputAmb);
      
      break;
  }
}

//long intervalDisplay=3000,lastDisplay=0,displayTimer1=0,displayTimer2=0,_numMesurement=30;

//var glob stato
void _display(){
  if(displayTimer1==0 && displayTimer2==0)
   displayTimer2=millis()+intervalDisplay;
  switch(stato){
    case STATE::IDLE:
        
      if(displayTimer1==0){//diplayTimer1 mostra a che fase è del singolo stato mentre displayTimer2 indica quale informazione stampare a video
         lcd.setCursor(0,0); //prima riga
         lcd.print(F("Stato: IDLE"));
         lcd.setCursor(0,1); //prima riga
         lcd.print(F("Premi il bottone NEXT"));
         lcd.setCursor(0,2); //prima riga
         lcd.print(F("per sett. la temp"));
         if(millis() - displayTimer2 >= intervalDisplay)
            displayTimer1=1;
      }else{
        lcd.setCursor(0,0); //prima riga
        lcd.print(F("Ta: "));
        lcd.print(tAmb.value,1);
        lcd.setCursor(12,0);
        lcd.print(F("Tap:"));
        lcd.print(buttonAmb.value,0);
        lcd.setCursor (0,1);//seconda riga 
        lcd.print(F("Tc:  "));
        lcd.print(tPlate.value,1);
        lcd.setCursor(12,1);
        lcd.print(F("Tcp:"));
        lcd.print(buttonPlate.value,0);
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
          lcd.print(buttonAmb.value,0);
          lcd.setCursor (0,1);//seconda riga 
          lcd.print(F("Tc:  "));
          lcd.print(tPlate.value,1);
          lcd.setCursor(12,1);
          lcd.print(F("Tcp:"));
          lcd.print(buttonPlate.value,0);
          if(millis() - displayTimer2 >= intervalDisplay){
            displayTimer1=2;
            displayTimer2=millis();
            lcd.clear();
           }
        }else if (displayTimer1==2){
          lcd.setCursor(0,0); //prima riga
          lcd.print(F("Ta: "));
          lcd.print(tAmb.value,1);
          lcd.setCursor(12,0);
          lcd.print(F("Tap:"));
          lcd.print(buttonAmb.value,0);
          lcd.setCursor (0,1);//seconda riga 
          lcd.print(F("Tc:  "));
          lcd.print(tPlate.value,1);
          lcd.setCursor(12,1);
          lcd.print(F("Tcp:"));
          lcd.print(buttonPlate.value,0);
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
  
  if(intervalDisplay<=millis()-lastDisplay){
  
    lastDisplay=millis();
  }
}
  /*S
   
   erial.print("Sto per avviare l'interfaccia num: ");
  Serial.println(state);
  delay(1000);
  long temp,tempCycle;
  
  switch (state){
      case 0:
              temp = millis();
              while(millis() - temp <= 10000){
                Serial.print("Valore del wrapper: ");
                Serial.print((btnPDOWN.getState()==Button::STATE::DOWN)?1:0);
                Serial.print(" ,ho terminato la lettura in: ");
                Serial.println( millis() - tempCycle);
                 tempCycle= millis();
              }
              break;
              
      case -1://wrapper button amb4
              temp = millis();
              while(millis() - temp <= 10000){
                Serial.print("Valore del wrapper: ");
                Serial.print(buttonPlate.value);
                Serial.print(" ora leggo il valore");
                buttonPlate.checkChange();
                Serial.print(" ,ho terminato la lettura in: ");
                Serial.println( millis() - tempCycle);
                 tempCycle= millis();
              }
              break;
      case 1:temp = millis();
              temp = millis();
              while(millis() - temp <= 10000){
                Serial.print("Valore del wrapper: ");
                Serial.print(buttonAmb.value);
                Serial.print(" ora leggo il valore");
                buttonAmb.checkChange();
                Serial.print(" ,ho terminato la lettura in: ");
                Serial.println( millis() - tempCycle);
                 tempCycle= millis();
              }
              break;
          
  }
 /* switch (state) {
     case RING:
      Serial.print("Testo il piatto riscaldante");
      Serial.print(RING);
      Serial.println("Controlla la temperatura, tieni le dita sul piatto e percepisci l'aumento di temp.... accendo l'anello");
      ring.set(125);
      for (int i = 0 ; i < 10 ; i++) {
        Serial.print("La temperatura della sonda è: ");
        Serial.println(tRing.getTemp());
        delay(750);

      }
      ring.set(0);
      Serial.println("La temperatura è cambiata ? 1 per affermativo 0 per alt");
      alert(state);
      state = PLATE;
      break;
    case PLATE:
      Serial.print("Testo il piatto riscaldante");
      Serial.print(PLATE);
      Serial.println("Controlla la temperatura, tieni le dita sul piatto e percepisci l'aumento di temp.... accendo il piatto");
      plate.set(125);
      for (int i = 0 ; i < 10 ; i++) {
        Serial.print("La temperatura della sonda è: ");
        Serial.println(tPlate.getTemp());
        delay(750);
      }
      plate.set(0);
      Serial.println("La temperatura è cambiata ? 1 per affermativo 0 per alt");
      alert(state);
      state = FRIDGE;
      break;
    case FRIDGE:
      Serial.println("frigo..");
      for (int i = 0; i < 4; i++) {
        fridge.turn(ON);
        Serial.println("on");
        delay(5000);
        fridge.turn(OFF);
        Serial.println("off");
        delay(3000);
      }
      alert(state);
      state = BTN_TEMP_PLATE_UP;
      break;
    case BTN_TEMP_PLATE_UP:
      checkButton(BTN_TEMP_PLATE_UP);
      state = BTN_TEMP_PLATE_DOWN;
      break;
    case BTN_TEMP_PLATE_DOWN:
      checkButton(BTN_TEMP_PLATE_DOWN);
      state = BTN_TEMP_AMB_UP;
      break;
    case BTN_TEMP_AMB_UP:
      checkButton(BTN_TEMP_AMB_UP);
      state = BTN_TEMP_AMB_DOWN;
      break;
    case BTN_TEMP_AMB_DOWN:
      checkButton(BTN_TEMP_AMB_DOWN);
      state = TEMP_AMB;
      break;
    case TEMP_AMB:
      Serial.print("La temperatura della sonda collegata al pin ");
      Serial.print(TEMP_AMB);
      Serial.print(" è di : ");
      Serial.println(tAmb.getTemp());
      Serial.println("Controlla la temperatura, tieni le dita sul sensore e osserva lo stato");
      for (int i = 0 ; i < 10 ; i++) {
        Serial.print("La temperatura della sonda è: ");
        Serial.println(tAmb.getTemp());
      }
      Serial.println("La temperatura è cambiata ? 1 per affermativo 0 per alt");
      state = TEMP_PLATE;
      break;

    case TEMP_PLATE:
      Serial.print("La temperatura della sonda collegata al pin ");
      Serial.print(TEMP_PLATE);
      Serial.print(" è di : ");
      Serial.println(tPlate.getTemp());
      Serial.println("Controlla la temperatura, tieni le dita sul sensore e osserva lo stato");
      for (int i = 0 ; i < 10 ; i++) {
        Serial.print("La temperatura della sonda è: ");
        Serial.println(tPlate.getTemp());
      }
      Serial.println("La temperatura è cambiata ? 1 per affermativo 0 per alt");
      state = TEMP_RING;
      break;
    case TEMP_RING:
      Serial.print("La temperatura della sonda collegata al pin ");
      Serial.print(TEMP_RING);
      Serial.print(" è di : ");
      Serial.println(tPlate.getTemp());
      Serial.println("Controlla la temperatura, tieni le dita sul sensore e osserva lo stato");
      for (int i = 0 ; i < 10 ; i++) {
        Serial.print("La temperatura della sonda è: ");
        Serial.println(tPlate.getTemp());
        delay(750);
      }
      Serial.println("La temperatura è cambiata ? 1 per affermativo 0 per alt");
      alert(state);
      state = DISPLAY;
      break;
    case DISPLAY:
      Serial.print("Sto per testare il display");
       lcd.setCursor(0,0); //prima riga
  lcd.print("Ta: ");
   lcd.setCursor(12,0);
  lcd.print("Tap:");
  lcd.setCursor (0,1);//seconda riga 
  lcd.print("Tc:  ");
      Serial.println("il display ha mostrato delle scrittte ?  ? 1 per affermativo 0 per alt");
      alert(state);
      state = -1;
      break;
    default: 
      Serial.println("TEST FINITO!");



  }*/



void alert(int num) {
  char res = '\n' ;

  while (1) {
    Serial.print(".");
    delay(200);
    if (Serial.available() > 0) {

      res = Serial.read();
      if (res == '0' || res == '1' )
        break;
    }
  }
  if (res == '1')
    return;
  else {
    while (1) {
      Serial.print(F("Errore analizzando elemento: "));
      Serial.println(num);
    }
  }
}


