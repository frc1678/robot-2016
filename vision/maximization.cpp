#include "maximization.h"
#include <iostream>

Maximization::Maximization(const ThingToMaximize* func) {
  f=func;
}

std::vector<double> Maximization::getVals() {
  return vals;
}

double Maximization::maximize() {
  vals=f->getStartEstimate();
  for(int i=0; i<f->getNumInputs(); i++) {
    std::cout<<"iteration" << i <<std::endl;
    bool hasImprovement;
    do {
     hasImprovement=false;
      for(int j=0; j<f->getNumInputs(); j++) {
        double scoreBefore, scoreAfter;
        std::vector<double> temp=vals;
        scoreBefore=f->getScore(temp, i);
        temp[j]+=f->getStepSize(i);
        scoreAfter=f->getScore(temp, i);
        if(scoreAfter<=scoreBefore) {
          temp[j]-=2*f->getStepSize(i);
          scoreAfter=f->getScore(temp, i);
          if(scoreAfter<=scoreBefore) {
            temp[j]+=f->getStepSize(i);
          }
          else {
            hasImprovement=true;
          }

        }
        else {
          hasImprovement=true;
        }
        vals=temp;
      }
      std::cout<<"score "<<f->getScore(vals, i)<<std::endl;
    } while(hasImprovement);
  }
  return f->getScore(vals, f->getNumIterations());
}
