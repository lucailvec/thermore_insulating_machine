#include "Fridge.h"
#include "Plate.h"
#include "TempSensor.h"
#include "Button.h"
#include <PCF8574_HD44780_I2C.h>
#define ONE_WIRE_BUS 17    

const int FRIDGE = 2;
const int BTN_TEMP_PLATE_UP = 3;
const int BTN_TEMP_PLATE_DOWN = 4;
const int BTN_TEMP_AMB_UP = 5;
const int BTN_TEMP_AMB_DOWN = 6;
const int BTN_RESET = 7;
const int PLATE = 9;
const int RING = 10;

const int TEMP_PLATE = A0;
const int TEMP_RING = A1;
const int TEMP_AMB = A2;

int state = RING;

Fridge fridge(FRIDGE);
Plate plate(PLATE, 255);
Plate ring(RING, 125);

Button btnPUP(BTN_TEMP_PLATE_UP, 200);
Button btnPDOWN(BTN_TEMP_PLATE_DOWN, 200);
Button btnAUP(BTN_TEMP_AMB_UP, 200);
Button btnADOWN(BTN_TEMP_AMB_DOWN, 200);

#define RNOMINALE 100000
#define TNOMINALE 25
#define BCOEFF 3955

// TempSensor(int numPin, int B, int R, int R0, float T0);
TempSensor tAmb(TEMP_AMB, BCOEFF, 99900, 100000, 25);
TempSensor tPlate(TEMP_PLATE, BCOEFF, 99800, 100000, 25);
TempSensor tRing(TEMP_RING, BCOEFF, 297000, 100000, 25);

#include <PCF8574_HD44780_I2C.h>
#define ONE_WIRE_BUS 17    


PCF8574_HD44780_I2C lcd(0x27,20,4);
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
  setOutput();
  Serial.begin(115200);
   lcd.init();                        
  lcd.backlight();      
  lcd.clear();
}

void setOutput() {
  pinMode(BTN_TEMP_PLATE_UP, OUTPUT);
  pinMode(BTN_TEMP_PLATE_DOWN, OUTPUT);
  pinMode(BTN_TEMP_AMB_UP, OUTPUT);
  pinMode(BTN_TEMP_AMB_DOWN, OUTPUT);
  pinMode(BTN_RESET, OUTPUT);

  fridge.turn(OFF);
  plate.set(0);
  ring.set(0);

  Serial.println(F("il test sta per incominciare, invia 1 per esito positivo 0 altrimenti"));
}
void loop() {
  Serial.print("Sto per testare l'interfaccia num: ");
  Serial.println(state);
  delay(1000);
  switch (state) {
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
      plate.set(255);
      for (int i = 0 ; i < 40 ; i++) {
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



  }
}

void checkButton(int btn) {
  Button::STATE statoBottone;

  switch (btn) {
    case BTN_TEMP_PLATE_UP:
      statoBottone = btnPUP.getState();
      break;
    case BTN_TEMP_PLATE_DOWN:
      statoBottone = btnPDOWN.getState();
      break;
    case BTN_TEMP_AMB_UP:
      statoBottone = btnAUP.getState();
      break;
    case BTN_TEMP_AMB_DOWN:
      statoBottone = btnADOWN.getState();
      break;
  }

  Serial.print("lo stato del bottone è : ");
  if(Button::STATE::DOWN==statoBottone)
    Serial.println("Premuto");
  else
    Serial.println("non premuto");
  Serial.println("Premi il bottone per eseguire il test");

  delay(3500);

  Button::STATE statoBottone2 = statoBottone;

  while (statoBottone2 == statoBottone) {
    Serial.print(".");
    delay(500);
    switch (btn) {
      case BTN_TEMP_PLATE_UP:
        statoBottone2 = btnPUP.getState();
        break;
      case BTN_TEMP_PLATE_DOWN:
        statoBottone2 = btnPDOWN.getState();
        break;
      case BTN_TEMP_AMB_UP:
        statoBottone2 = btnAUP.getState();
        break;
      case BTN_TEMP_AMB_DOWN:
        statoBottone2 = btnADOWN.getState();
        break;
    }
  }
  Serial.println("");
  Serial.print("Hai premuto il tasto e portato il livello a : ");
  if(Button::STATE::DOWN==statoBottone)
    Serial.println("Premuto");
  else
    Serial.println("non premuto");
}

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


