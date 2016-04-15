#ifndef _MIXIMIZATION_H_
#define _MAXIMIZATION_H_

#include <vector>

class ThingToMaximize {
public:
  virtual int getNumInputs() const =0;
  virtual int getNumIterations() const =0;
  virtual double getScore(std::vector<double> intputs, int iteration) const =0;
  virtual std::vector<double> getStartEstimate()const =0;
  virtual double getStepSize(int iteration)const =0;
};

class Maximization {
public:
  Maximization(const ThingToMaximize *func);
  std::vector<double> getVals();
  double maximize();
protected:
  const ThingToMaximize *f;
  std::vector<double> vals;
};

#endif
