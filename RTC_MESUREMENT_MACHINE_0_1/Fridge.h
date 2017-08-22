


#ifndef Fridge_H
#define Fridge_H




class Fridge
{
public: 
  enum STATE { ON , OFF  };
  Fridge(int pin);
  //Fridge(int pin, bool _onState);
  void turn(STATE st);
private:
  int _numPin;
  bool _onState;
  
};

#endif

