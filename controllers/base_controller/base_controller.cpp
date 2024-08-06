/**
 * @file    cameras_example.cpp
 * @brief   Controller example for controlling the cameras of the robot.
 *
 * @author  Raul Perula-Martinez <raul.perula@uc3m.es>
 * @author  Javier Pastor Fernandez <javpasto@ing.uc3m.es>
 * @date    2021-12  
 */

#include "MyRobot.h"

 /**
  * @brief Main program.
  */
int main(int argc, char** argv)
{
    MyRobot* my_robot = new MyRobot();

    my_robot->run();

    delete my_robot;

    return 0;
}
