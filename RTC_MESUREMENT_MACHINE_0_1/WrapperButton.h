

#ifndef WrapperButton_H
#define WrapperButton_H

#ifndef Arduino_H
#include "Arduino.h"
#endif

#ifndef Button_H
#include "Button.h"
#endif

class WrapperButton{
  public: WrapperButton(int initialValue, Button *increment, Button *decrement);//un costruttore che consente di inizializzare il valore ad una certa soglia e in più che acquisisca i puntatori dei bottoni da utilizzare
            int value;//devo esporre pubblicamente lo stato di una variabile
          //posso scegliere una funzione che mi va sempre a richiamare i bottoni (quindi pesa sul processore) e ritorna un valore
          //oppure posso scegliere una funzione meno prona ad errori ma che mi consente di dividere la parte di aggiornamento del valore ad una di lettura
          void checkChange();
  private: Button * _increment;
           Button * _decrement; //devo andare a controllare lo stato dei componenti per poter aggiornare il valore
  
};

#endif
