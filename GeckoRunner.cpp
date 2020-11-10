// File:          GeckoRunner.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include<ctime>
#include<iostream>
#include<math.h>
#include<Eigen/Core> 

//#include "Motion.hpp"
#include "motion_control.cpp" 
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/Accelerometer.hpp>
#include <webots/InertialUnit.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace Eigen;
using namespace std;


//extern float limb_force_sensor[4];

Motion_control mc;
//virtual enable (timeStep);
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
/*  // delay
static void sleep_ms(unsigned int secs)
{
struct timeval tval;

tval.tv_sec=secs/1000;

tval.tv_usec=(secs*1000)%1000000;

select(0,NULL,NULL,NULL,&tval);
}
*/

// This fution used to delay ms in linux environment 
int main(int argc, char **argv) {
  mc.webots_init();
  
  for(int i=0;i<4;i++)
    {
      for(int j=0;j<3;j++)
        {
          mc.limb_motors[i][j]->setPosition(0);
        }
    }
  
  // get the time step of the current world.
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (mc.robot->step(mc.timeStep) != -1) {

    //never end 99
    /*
    mc.webots_relate();
    mc.posture_vel_update();
    mc.joint_vel_update();
    mc.four_feet_position_update();
    cout << mc.four_feet_position << endl;
    */
      
      
      mc.four_feet_motion();
      
      
    
    
    

    
   
    //  motion_controller.Outputall_PWM();
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.
    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.
  cout << "no happy" << endl;
  delete mc.robot;
  return 0;
}
