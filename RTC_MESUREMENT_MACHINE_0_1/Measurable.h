
#ifndef MEASURABLE_H
#define MEASURABLE_H

class Measurable{
       public:
          double mean();
          double sd();
          void reset();//double output; ce l'ho già all'esterno
          void newVal(double val);
          int _numOfValue;
       private: double _sum;
                double _sumAt2;
                
               // double _mean;
               // double _sd;
              //  unsigned int _lastMean;
               // unsigned int _lastSd;
};

#endif
