#include "Controller.h"

Controller::Controller()
{
    
}

void Controller::begin()
{
    arm.begin();
    comm.begin();
    comm.serialCmdInit();
}

void Controller::readCommand()
{
    comm.getSerialCmd();
}
void Controller::move()
{
    // Implement the logic to move the robotic arm based on the command received
    // For example, you can call arm.moveToXYZ() with the desired position
}