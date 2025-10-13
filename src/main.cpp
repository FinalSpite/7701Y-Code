/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       njwolff                                                   */
/*    Created:      12/7/2024, 10:05:06 PM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include <string.h>

using namespace vex;
using signature = vision::signature;
using code = vision::code;
// A global instance of competition
competition Competition;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

//since the pre-auton function is treaded we need to make sure that it is done before the buttons are placed on the screen
bool running = true;

//if the auton is on practice mode it will wait a second before running.
bool practice = false;

int autonchoice = 0;
//when auton = 0 is left side code when = 1 its right side code and when =2 its practice driving.

// define your global instances of motors and other devices here
// VEXcode device constructors
controller Controller1 = controller(primary);
motor leftMotorA = motor(PORT14, ratio6_1, false);
motor leftMotorB = motor(PORT15, ratio6_1, true);
motor leftMotorC = motor(PORT16, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT11, ratio6_1, true);
motor rightMotorB = motor(PORT12, ratio6_1, false);
motor rightMotorC = motor(PORT13, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT17);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 259.34, 289.56, 254, mm, 1.3333333333333);
motor intakeMotor = motor(PORT7, ratio6_1, true);
motor intakeMotor2 = motor(PORT8, ratio18_1, false);
motor_group allIntake = motor_group(intakeMotor, intakeMotor2);
digital_out sol1 = digital_out(Brain.ThreeWirePort.H);
digital_out sols2 = digital_out(Brain.ThreeWirePort.G);





/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/ 

void pre_auton(void) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  Brain.Screen.print("Calibrating Inertial for Drivetrain");
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  wait(250, msec);
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  Brain.Screen.clearScreen();
  running = false;
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control the robot during the autonomous phase of    */
/*  the VEX Competition.                                                     */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/ 

// Code for left side autonomous (basic)
void autonomous_left(void){
  Brain.Screen.clearScreen();
  if(practice == true){
    wait(1500, msec);
  }

  Drivetrain.setDriveVelocity(50, percentUnits::pct);
  Drivetrain.drive(forward);
  wait(500, msec);
  Drivetrain.stop();
}

// Code for right side autonomous (basic)
void autonomous_right(void) {
  Brain.Screen.clearScreen();
  if(practice == true){
    wait(1500, msec);
  }
  intakeMotor.spin(forward, 100, pct);
  LeftDriveSmart.spin(reverse);
  RightDriveSmart.spin(forward);
  wait(1000, msec);
  Drivetrain.stop();
  wait(1500, msec);
  intakeMotor.stop();


}

void autonomous_start( void ){
  Controller1.Screen.clearScreen();
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(0,0,230,272);
  Brain.Screen.printAt(10,45,"LEFT");

  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(250,0,230,272);
  Brain.Screen.printAt(260,45, "RIGHT");


  while(true){
    Controller1.Screen.setCursor(0,0);
    Controller1.Screen.print("Ready");
    if ((Brain.Screen.pressing() == true)||(Controller1.ButtonLeft.pressing()==true)||(Controller1.ButtonRight.pressing()==true)){
      int x = Brain.Screen.xPosition();
      
      if ((x<=230)||(Controller1.ButtonY.pressing()==true)){
        practice = true;
        autonomous_left();
      }else if((x>=250)||(Controller1.ButtonX.pressing()==true)){
        practice = true;
        autonomous_right();
      }
    }
  }
} 

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*                                                                           */
/*      The Code For the driver control is the in the robot-config file      */
/*           under the function rc_auto_loop_function_Controller1            */
/*-------------------------------------\/------------------------------------*/ 

void user(void) {
  // User control code here, inside the loop
    Brain.Screen.setFillColor(black);
    Brain.Screen.print("User Code");
    thread auto_loop_controller_brain(auto_loop_controller_brain_update);
    wait(200,msec);
    thread rc_auto_loop_controller(rc_auto_loop_function_Controller1);
    Brain.Screen.newLine();
    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//

int main() {
  // Set up callbacks for autonomous and driver control periods.

  // Run the pre-autonomous function.
  pre_auton();

  run();
  
  Brain.Screen.setFillColor(black);

 // Set up callbacks for autonomous and driver control periods based on button presses on the run screen.
  if (autonchoice == 0){
    Competition.autonomous(autonomous_left);
    Competition.drivercontrol(user);
  } else if (autonchoice == 1){
    Competition.autonomous(autonomous_right);
    Competition.drivercontrol(user);
  } else if (autonchoice == 2){
    user();
  } else if(autonchoice == 3){
    wait(1500,msec);
    autonomous_start();
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
