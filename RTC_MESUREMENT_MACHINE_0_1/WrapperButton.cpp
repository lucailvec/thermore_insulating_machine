
#include "WrapperButton.h"

#ifndef Button_H
#include "Button.h"
#endif

WrapperButton::WrapperButton(double initialValue, Button *increment, Button *decrement){
  value = initialValue;
  _increment = increment;
  _decrement = decrement;
}

// Viene aggiunto l'aspetto dinamico ottenuto tramite una chiamata che va ad aggiornare lo stato della variabile grazie all'ispezione dei bottoni

void WrapperButton::checkChange(){
  if(_increment->getState() == Button::STATE::DOWN)
    value++;

  if(_decrement->getState() == Button::STATE::DOWN)
    value--;
}

