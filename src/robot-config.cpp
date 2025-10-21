#include "vex.h"
#include <string.h>
#include <iostream>

using namespace vex;
using signature = vision::signature;
using code = vision::code;

bool RemoteControlCodeEnabled = true;

// define variables used for controlling motors based on controller inputs
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool sol1ToggleLast =false;
bool sol1Toggle = true;
bool sol2ToggleLast =false;
bool sol2Toggle = true;

//These booleans are used so only one button has to be pressed to do an action
//This is because button presses are detected many times in a second, so it would just go on and off without bound.

float Kp = 1.75;   // Proportional gain only for LB PID
float Kpp = 0.4;   // Proportional gain for PID
float Ki = 0.0;   // Integral gain for PID
float Kd = 0.45;   // Derivative gain for PID
float driveangle;

/*------------------------------------------------*/
/*                                                */
/*      Controller and Brain Screen Updating      */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/



/*------------------------------------------------*/
/*                                                */
/*               UserDrive Code                   */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/
// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  Drivetrain.setDriveVelocity(85, percent);
  while(true) {
    if(RemoteControlCodeEnabled) {
      // stop the motors if the brain is calibrating
      if (DrivetrainInertial.isCalibrating()) {
        LeftDriveSmart.stop();
        RightDriveSmart.stop();
        while (DrivetrainInertial.isCalibrating()) {
          wait(25, msec);
        }
      }
      Drivetrain.setStopping(coast);
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis1 - Axis3
      // right = Axis1 + Axis3
      int drivetrainLeftSideSpeed = Controller1.Axis1.position() + Controller1.Axis3.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      int deadzone = 9;

      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < deadzone && drivetrainLeftSideSpeed > -deadzone) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < deadzone && drivetrainRightSideSpeed > -deadzone) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }


      // This section is for button holds and toggles for other actions on the robot (solenoids, intake, etc)
      

      if ((Controller1.ButtonA.pressing() == true)){
        if (sol1ToggleLast == false){
          if (sol1Toggle == true){
            sol1.set(true); 
            sol1Toggle = false;
          }
          else if (sol1Toggle == false)
          {
            sol1.set(false);
            sol1Toggle = true;
          } 
          sol1ToggleLast = true;
        }
      }
      if (Controller1.ButtonA.pressing() == false){
        sol1ToggleLast = false;
      }

      if ((Controller1.ButtonY.pressing() == true)){
        if (sol2ToggleLast == false){
          if (sol2Toggle == true){
            sols2.set(true); 
            sol2Toggle = false;
          }
          else if (sol2Toggle == false)
          {
            sols2.set(false);
            sol2Toggle = true;
          } 
          sol2ToggleLast = true;
        }
      }
      if (Controller1.ButtonY.pressing() == false){
        sol2ToggleLast = false;
      }


      if ((Controller1.ButtonR1.pressing() == true)){
          intakeMotor.spin(reverse, 100, percent);
          intakeMotor2.spin(reverse, 70, percent);
      }
      if ((Controller1.ButtonR2.pressing() == true)){
          intakeMotor.spin(forward, 100, percent);
          intakeMotor2.spin(forward, 100, percent);
        }
      if ((Controller1.ButtonL1.pressing() == true)){
          intakeMotor.spin(reverse, 100, percent);
      }
      if ((Controller1.ButtonL2.pressing() == true)){
          intakeMotor.spin(forward, 100, percent);
        }
      if ((Controller1.ButtonR1.pressing() == false)&&(Controller1.ButtonR2.pressing()==false)&&(Controller1.ButtonL1.pressing() == false)&&(Controller1.ButtonL2.pressing()==false)){
          intakeMotor.stop();
          intakeMotor2.stop();
      }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
}


/*
       Template for toggle controls

      if ((Controller1.ButtonA.pressing() == true)){
        if (toggleLast == false){
          if (toggle == true){
            Sol1.set(true); 
            toggle = false;
          }
          else if (toggle == false)
          {
            Sol1.set(false);
            toggle = true;
          } 
          toggleLast = true;
        }
      }
      if (Controller1.ButtonA.pressing() == false){
        toggleLast = false;
      }


*/
/*----------------------/\------------------------*/
/*                                                */
/*               UserDrive Code                   */
/*                    ||||                        */
/*                                                */
/*------------------------------------------------*/




/*------------------------------------------------*/
/*                                                */
/*                     PID                        */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/
void turn_to_angle(float targetAngle) {
    float error;            // Difference between target and current angle
    float previousError = 0;
    float integral = 0;
    float derivative;
    float motorPower;

    while (true) {
      
        // Calculate the current error
        error = targetAngle - DrivetrainInertial.rotation();
        driveangle = DrivetrainInertial.rotation();
        // Proportional term
        float P = Kpp * error;
        
        // Integral term
        integral += error;
        float I = Ki * integral;
        
        // Derivative term
        derivative = error - previousError;
        float D = Kd * derivative;
        
        float KL = 1;

        // PID output
        motorPower = P + I + D;

        // Set motor powers (adjust this for left and right turns)
        LeftDriveSmart.spin(directionType::fwd, motorPower, velocityUnits::pct);
        RightDriveSmart.spin(directionType::rev, motorPower, velocityUnits::pct);
        

        // Update previous error
        previousError = error;
        
        // Break loop if error is within a small threshold
          if (fabs(error) < KL) {
            break;
          }
          
        
        // Small delay to avoid overloading the CPU
        task::sleep(20);
    }
    
    // Stop motors once the target angle is reached
    LeftDriveSmart.stop();
    RightDriveSmart.stop();
}


/*------------------------------------------------*/
/*                                                */
/*           Buttons on the Screeen               */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/


void run( void ) {
  if(running != true){
  Drivetrain.setStopping(coast);
  RightDriveSmart.setVelocity(30, percentUnits::pct);
  LeftDriveSmart.setVelocity(30, percentUnits::pct);
  wait(500,msec);
  // Create buttons for auto or driver control selection
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0,0,230,140);
    Brain.Screen.setFont(prop30);
    Brain.Screen.printAt(30,77,"LEFT AUTON");
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(250,0,230, 140);
    Brain.Screen.printAt(276,77,"RIGHT AUTON");
    Brain.Screen.setFillColor(yellow);
    Brain.Screen.drawRectangle(0,150, 230,110);
    Brain.Screen.printAt(40, 200, "USER CODE");
    Brain.Screen.setFillColor(orange);
    Brain.Screen.drawRectangle(250,150, 230, 110);
    Brain.Screen.printAt(270, 200, "AUTON PRACT");
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.print("       X: RIGHT    ");
    Controller1.Screen.newLine();
    Controller1.Screen.print("Y: LEFT        A: PRAC");
    Controller1.Screen.newLine();
    Controller1.Screen.print("       B: USER     ");
  }
  while (1)
  {
  // Check if and where the screen is pressed
  //Controller so you dont have to touch the brain.
    if(Controller1.ButtonB.pressing() == true){
      autonchoice = 2;
      Controller1.Screen.clearScreen();
      break;
    }
    if(Controller1.ButtonY.pressing() == true){
      autonchoice = 0;
      Controller1.Screen.clearScreen();
      break;
    }
    if(Controller1.ButtonX.pressing() == true){
      autonchoice = 1;
      Controller1.Screen.clearScreen();
      break;
    }
    if(Controller1.ButtonA.pressing() == true){
      autonchoice = 3;
      Controller1.Screen.clearScreen();
      break;
    }

    if (Brain.Screen.pressing()){
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();

      if ((x<=230)&&(y<=140)){
        autonchoice = 0;
        break;
      }else if((y<=140)&&(x>=250)){
        autonchoice = 1;
        break;
      }else if((y>=150)&&(x<=230)){
        autonchoice = 2;
        break;
      }else if((y>=150)&&(x>=250)){
        autonchoice = 3;
        break;
      }
    }
    wait(10, msec);
  }
  Brain.Screen.clearScreen();
  wait(20, msec);
}