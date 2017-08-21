#ifndef BUTTON_H
#define BUTTON_H

class Button{
  public:
    enum STATE { DOWN, UP};
    Button(int pin);
    Button (int pin, int interval);
    Button::STATE getState();
  private:
    int _pin;
    int _interval;
    unsigned long _lastRead;
};

#endif


