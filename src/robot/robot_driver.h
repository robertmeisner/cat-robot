/**
 * Cat Robot Driver Implementation with servos, distance sensor, IR remote and autonomous mode!
 *
 * @file motor_driver.h
 * @brief Robot Driver Implementation
 * @author Robert Meisner <robert@catchit.pl>
 * @version 1.0 2016-12-22
 */
#include "../motor/continous_servo_motor_driver.h"
#include "../distance/hc_sr04_distance_sensor_driver.h"
#include "../IR/keyes_ir_remote_driver.h"
// Pin assignment and servo declarations

namespace CatRobot
{
/**
* Structure describing robot's states
*/
enum State
{
    TURNING_LEFT,
    TURNING_RIGHT,
    MOVING_FORWARD,
    MOVING_BACKWARD,
    STOPPED
};
class Robot
{
  public:
    /*
         * @brief Class constructor.
         */
    Robot(int trigPin,
          int echoPin,
          int leftMotorPin,
          int rightMotorPin)
        : trigPin(trigPin), echoPin(echoPin), leftMotorPin(leftMotorPin), rightMotorPin(rightMotorPin)
    {
        //create distance sensor instance
        distanceSensor = new CatRobot::HC04DistanceSensorDriver(trigPin, echoPin, (long)maxDistance);
        //create left motor instance
        leftMotor = new CatRobot::ContinousServoMotorDriver(leftMotorPin);
        //create right motor instance
        rightMotor = new CatRobot::ContinousServoMotorDriver(rightMotorPin);
        //create remote instance
        remote = new CatRobot::KeyesIRRemoteDriver(12);
    }
    /**
    * Inits robot's components
     */
    void init()
    {
        // init drivers
        distanceSensor->init();
        leftMotor->init();
        rightMotor->init();
        remote->init();
        // if analog input pin 0 is unconnected, random analog
        // noise will cause the call to randomSeed() to generate
        // different seed numbers each time the sketch runs.
        // randomSeed() will then shuffle the random function.
        randomSeed(analogRead(0));
        move(0, 0);
    }
    /**
    * Explore function executes logic
    *
    * @return True if robot can continue the exploration, iforms that explore() can be called again, false when robot should finish its operation or a critical error has occured. No subsequent call to explore() should be performed.
    */
    bool explore()
    {

        if (operationTimeLeft > millis())
        {
            Serial.print("Time to finish: ");
            Serial.println(operationTimeLeft - millis());
        }
        else
        {
            //Serial.print("Time to finish: ");
            //Serial.println("Negative number");
        }
        //get distance
        unsigned long distance = distanceSensor->getDistance();
        bool obstacle = distance < tooClose;
        //IS MANUAL MODE SWITCH BUTTON PRESSED?
        bool keyPressed = remote->keyPressed();
        if (keyPressed && remote->isOK())
        {
            MANUAL_CONTROL = !MANUAL_CONTROL;
            move(0, 0);
            return true;
        }
        // PERFORM DIFFERENT BEHAVIOUR FOR MANUAL AND AUTONOMOUS MODE
        switch (MANUAL_CONTROL)
        {
        case true:

            if (((isMoving() && moveFinished()) || (isTurning() && turnFinished())))
            {
                //stop
                move(0, 0);
            }
            else
            {
                if (keyPressed)
                { //we have received an IR

                    if (remote->isUp())
                    {
                        move(255, 1000);
                    }
                    else if (remote->isDown())
                    {
                        move(-255, 1000);
                    }
                    else if (remote->isLeft())
                    {
                        turn(-30);
                    }
                    else if (remote->isRight())
                    {
                        turn(30);
                    }

                    Serial.println(remote->value(), HEX); //display HEX
                                                          //next value
                }
                else
                {
                    // do nothing
                }
            }

            break;
        case false:

            // We always check distance first
            if ((isMoving() && state != MOVING_BACKWARD) && obstacle)
            {
                //stop
                move(0, 0);
                // move back a bit
                move(-200, 1000 * random(1, 2));

            } //if just moved back rotate him
            else if (isMoving() && moveFinished() && state == MOVING_BACKWARD)
            {
                //turn semi randomly
                turn((random(1) ? -1 : 1) * 20 + random(10));

            } // if he just turned lets move forward
            else if (isTurning() && turnFinished())
            {
                move(255, 1000 * random(4, 10));
            }
            //if he just moved forward
            else if (isMoving() && moveFinished() && state != MOVING_BACKWARD)
            {
                turn((random(1) ? -1 : 1) * 20 + random(10));
            }
            //if we stopped for some reason lets turn
            else if (isStopped())
            {
                turn((random(1) ? -1 : 1) * 20 + random(10));
            }
            break;
        }
        return true;
    }
    /**
    * Checks if robot turns
    */
    bool isTurning()
    {
        return (state == TURNING_LEFT || state == TURNING_RIGHT);
    }
    /**
    * Checks if robots last action was turn and it has finished
    */
    bool turnFinished()
    {
        if (isTurning() && operationTimeLeft > millis())
        {
            return false;
        }
        return true;
    }
    /**
    * Checks if robot moves
    */
    bool isMoving()
    {
        return (state == MOVING_FORWARD || state == MOVING_BACKWARD);
    }
    /**
    * Checks if robots last action was move and it has finished
    */
    bool moveFinished()
    {
        if (isMoving() && operationTimeLeft > millis())
        {
            return false;
        }
        return true;
    }
    /**
    * Checks if robot stopped
    */
    bool isStopped()
    {
        return state == STOPPED;
    }
    /**
    * Turns robot left (angle<0) or right (angle>0) for a given angle.
    * @todo executing turn should use some sensor data - not only the timer
    * @param angle Angle robot should turn. For angle<0 turns left, for angle>0 turns right
    */
    void turn(int angle)
    {

        int timeToTurn90 = 500;
        int timetoTurn = timeToTurn90 * ((float)angle / (float)90);
        Serial.print("Turning angle: ");
        Serial.println(angle);
        Serial.print("Turning timetoTurn: ");
        Serial.println(timetoTurn);
        setOperationTime(timetoTurn);
        if (angle >= 0)
        {
            state = TURNING_RIGHT;
            leftMotor->setSpeed(-255);
            rightMotor->setSpeed(-255);
        }
        else
        {
            state = TURNING_LEFT;
            leftMotor->setSpeed(255);
            rightMotor->setSpeed(255);
        }

        //turn for
        //set state to TURNING
        //adjust turning time
    }
    /**
    * Moves robot
    * @param speed Speed robot should move
    * Valid values are between -255 and 255. 
    * Use positive values to run the motor forward, 
    * negative values to run it backward and zero to stop the motor.
    * @param timeToMove Time limit robot should move in that direction
    */
    void move(int speed, int timeToMove)
    {

        if (speed == 0)
        {
            state = STOPPED;
            Serial.print("Moving stopped: ");
        }
        else if (speed > 0)
        {
            state = MOVING_FORWARD;
            Serial.print("Moving forward: ");
        }
        else
        {
            state = MOVING_BACKWARD;
            Serial.print("Moving backward: ");
        }
        int leftSpeed = speed * -1;
        int rightSpeed = speed;

        setOperationTime(timeToMove);
        leftMotor->setSpeed(leftSpeed);
        rightMotor->setSpeed(rightSpeed);
        Serial.print(leftSpeed);
        Serial.print(" - ");
        Serial.print(rightSpeed);
        //Serial.println(speed);
        Serial.print("Moving timeToMove: ");
        Serial.println(timeToMove);
    }
    setOperationTime(long time)
    {
        operationTimeLeft = millis() + time;
    }

  private:
    //Arduino specific - probably should move it somewhere else
    int trigPin = 2;
    int echoPin = 4;
    int leftMotorPin = 9;
    int rightMotorPin = 8;

    State state;
    // MANUAL SWITCH ON/OFF
    bool MANUAL_CONTROL = true;
    // every operation is timed. this is timer for current operation
    unsigned long operationTimeLeft = 0;
    // the distance robot should keep from obstacles
    int tooClose = 200;
    // components declarations
    int maxDistance = tooClose * 10;
    CatRobot::HC04DistanceSensorDriver *distanceSensor = NULL;
    CatRobot::ContinousServoMotorDriver *leftMotor = NULL;
    CatRobot::ContinousServoMotorDriver *rightMotor = NULL;
    CatRobot::KeyesIRRemoteDriver *remote = NULL;
};
}