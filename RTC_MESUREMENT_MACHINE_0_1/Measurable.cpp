
#include "Measurable.h"

#ifndef MATH_H
#include <math.h>
#endif
/*          
 *        double mean();
          double sd();
          //double output; ce l'ho giàà all'esterno
       private: double _sum;
                double _sum^2;
                unsigned int _numOfValue;
                void _newVal(double val);
 */


double Measurable::mean(){
  return _sum/_numOfValue;
  
}

double Measurable::sd(){
  double meanAt2 = mean();//quadrato della media
  meanAt2*=meanAt2;
  //media dei quadrati

  return (1.0/_numOfValue)*sqrt(_numOfValue*_sumAt2-meanAt2);
}
void Measurable::newVal(double val){
  _numOfValue++;
  _sum+=val;
  _sumAt2 += val*val;
  
}

void Measurable::reset(){
  _numOfValue=0;
  _sum=0;
  _sumAt2=0;

}

