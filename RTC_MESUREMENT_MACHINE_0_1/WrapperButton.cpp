
#include "WrapperButton.h"

#ifndef Button_H
#include "Button.h"
#endif

WrapperButton::WrapperButton(int initialValue, Button *increment, Button *decrement,int min,int max){
  _min=min;
  _max=max;
  value = initialValue;
  dValue=value;
  _increment = increment;
  _decrement = decrement;
}

// Viene aggiunto l'aspetto dinamico ottenuto tramite una chiamata che va ad aggiornare lo stato della variabile grazie all'ispezione dei bottoni

void WrapperButton::checkChange(){
  if(_increment->getState() == Button::STATE::DOWN)
    value++;

  if(_decrement->getState() == Button::STATE::DOWN)
    value--;

  if(value>_max)
    value=_max;

  if(value<_min)
    value = _min;

  dValue=value;
}

void WrapperButton::set(int v){
  value = v;
  dValue=v;
}



