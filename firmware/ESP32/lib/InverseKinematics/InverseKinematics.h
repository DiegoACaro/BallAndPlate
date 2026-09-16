#ifndef InverseKinematics_H
#define InverseKinematics_H
#include "arduino.h"

//constants
#define LINK_A 0
#define LINK_B 1
#define LINK_C 2

class Machine { //machine class
  public:
    //class functions
    Machine(double d, double e, double f, double g);
    double theta(int leg, double hz, double nx, double ny); //returns the value of theta a, b, or c
};

#endif
