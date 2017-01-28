/**
 * @file cat_robot_distance.ino
 * @brief Arduino remotely controlled robot with autonomous mode 
 * @author Robert Meisner <robert@catchit.pl>
 */

#include "src/robot/robot_driver.h"
int trigPin = 2;
int echoPin = 4;
int leftMotorPin = 9;
int rightMotorPin = 8;

CatRobot::Robot robot(trigPin,
                     echoPin,
                     leftMotorPin,
                     rightMotorPin);
/**
 * Init the robot
 */
void setup() 
{
  Serial.begin(9600);
  robot.init();
}
/**
 * Program's loop function calling robot.explore().
 * Every call of robot.explore() function runs  robots behavior routines.
 */
void loop()
{

  while(robot.explore()){
    
  }
}
