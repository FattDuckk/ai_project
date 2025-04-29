#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "RoboticArm.h"
#include "SerialComm.h"

class Controller
{
    public:
        Controller();
        void begin();
        void readCommand();
        void move();
    
    private:
        SerialComm comm;
        RoboticArm arm;
        
};

#endif