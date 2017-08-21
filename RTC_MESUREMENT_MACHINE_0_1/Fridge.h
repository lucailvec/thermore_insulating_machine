


#ifndef Fridge_H
#define Fridge_H


enum State { ON , OFF  };

class Fridge
{
public: 
  Fridge(int pin);
  //Fridge(int pin, bool _onState);
  void turn(State st);
private:
  int _numPin;
  bool _onState;
  
};

#endif

