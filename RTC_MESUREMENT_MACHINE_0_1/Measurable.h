
#ifndef MEASURABLE_H
#define MEASURABLE_H
#include "QuickStats.h"
class Measurable{
       public:
          Measurable(long interval,double * ar,int num);
          double median();
          double cv();
          void newVal(double val);
          int NUMSAMPLES;
          double median(int num);
          double cv(int num);
          void reset();
       private: QuickStats * _stats;
                long _interval;
                long _lastRead;
                double * _ar;
                unsigned int _counter;
                void _newVal(double val);
                
               // double _mean;
               // double _sd;
              //  unsigned int _lastMean;
               // unsigned int _lastSd;
};

#endif


