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
motor leftMotorA = motor(PORT11, ratio6_1, false);
motor leftMotorB = motor(PORT12, ratio6_1, true);
motor leftMotorC = motor(PORT13, ratio6_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT14, ratio6_1, true);
motor rightMotorB = motor(PORT15, ratio6_1, false);
motor rightMotorC = motor(PORT16, ratio6_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT17);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 259.34, 289.56, 254, mm, 1.3333333333333);
motor intakeMotor = motor(PORT5, ratio6_1, true);
motor intakeMotor2 = motor(PORT4, ratio18_1, false);
motor_group allIntake = motor_group(intakeMotor, intakeMotor2);
digital_out sol1 = digital_out(Brain.ThreeWirePort.H);
digital_out sols2 = digital_out(Brain.ThreeWirePort.G);


rotation XTracking = rotation(PORT18, true);
rotation YTracking = rotation(PORT19, false);




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
  turn_to_angle(-20);
  allIntake.setVelocity(100, percent);
  intakeMotor.spin(reverse);
  Drivetrain.setDriveVelocity(60, percent);
  Drivetrain.drive(forward);
  wait(800, msec);
  Drivetrain.stop();
  wait(600, msec);
  intakeMotor.stop();
  turn_to_angle(-143);
  wait(30, msec);
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(forward);
  wait(1300, msec);
  intakeMotor.stop();
  Drivetrain.stop();
  turn_to_angle(180);
  wait(30, msec);
  Drivetrain.setDriveVelocity(50,percent);
  Drivetrain.drive(reverse);
  wait(600, msec);
  allIntake.spin(reverse);
  wait(1500, msec);
  allIntake.stop();
  Drivetrain.stop();
  sol1.set(true);
  Drivetrain.setDriveVelocity(50, pct);
  intakeMotor.spin(reverse, 100, percent);
  Drivetrain.drive(forward);
  wait(400, msec);
  turn_to_angle(180);
  wait(40, msec);
  Drivetrain.drive(forward);
  wait(1300, msec);
  Drivetrain.drive(reverse);
  wait(250, msec);
  Drivetrain.setDriveVelocity(30, percent);
  Drivetrain.drive(forward);
  wait(400, msec);
  Drivetrain.stop();
  wait(600, msec);
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(reverse);
  wait(200, msec);
  turn_to_angle(-175);
  Drivetrain.drive(reverse);
  wait(900, msec);
  allIntake.spin(reverse);
  wait(400, msec);
  Drivetrain.stop();
  wait(800, msec);
  Drivetrain.drive(forward);
  wait(300, msec);
  Drivetrain.stop();
  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.drive(reverse);

}

// Code for right side autonomous (basic)
void autonomous_right(void) {
  turn_to_angle(20);
  allIntake.setVelocity(100, percent);
  intakeMotor.spin(reverse);
  Drivetrain.setDriveVelocity(60, percent);
  Drivetrain.drive(forward);
  wait(800, msec);
  Drivetrain.stop();
  wait(600, msec);
  intakeMotor.stop();
  turn_to_angle(143);
  wait(30, msec);
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(forward);
  wait(1300, msec);
  intakeMotor.stop();
  Drivetrain.stop();
  turn_to_angle(180);
  wait(30, msec);
  Drivetrain.setDriveVelocity(50,percent);
  Drivetrain.drive(reverse);
  wait(600, msec);
  allIntake.spin(reverse);
  wait(1500, msec);
  allIntake.stop();
  Drivetrain.stop();
  sol1.set(true);
  Drivetrain.setDriveVelocity(50, pct);
  intakeMotor.spin(reverse, 100, percent);
  Drivetrain.drive(forward);
  wait(400, msec);
  turn_to_angle(180);
  wait(40, msec);
  Drivetrain.drive(forward);
  wait(1300, msec);
  Drivetrain.drive(reverse);
  wait(250, msec);
  Drivetrain.setDriveVelocity(30, percent);
  Drivetrain.drive(forward);
  wait(400, msec);
  Drivetrain.stop();
  wait(600, msec);
  Drivetrain.setDriveVelocity(50, percent);
  Drivetrain.drive(reverse);
  wait(200, msec);
  turn_to_angle(175);
  Drivetrain.drive(reverse);
  wait(900, msec);
  allIntake.spin(reverse);
  wait(400, msec);
  Drivetrain.stop();
  wait(800, msec);
  Drivetrain.drive(forward);
  wait(300, msec);
  Drivetrain.stop();
  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.drive(reverse);

}

void autonomous_start( void ){
  Controller1.Screen.clearScreen();
  Brain.Screen.setFillColor(green);
  Brain.Screen.drawRectangle(0,0,230,272);
  Brain.Screen.printAt(10,45,"LEFT");

  Brain.Screen.setFillColor(red);
  Brain.Screen.drawRectangle(250,0,230,272);
  Brain.Screen.printAt(260,45, "RIGHT");
  Controller1.Screen.setCursor(0,0);
  Controller1.Screen.print("                       ");
  Controller1.Screen.newLine();
  Controller1.Screen.print("Y: LEFT         A: RIGHT");
  Controller1.Screen.newLine();
  Controller1.Screen.print("                       ");


  while(true){
    if ((Brain.Screen.pressing() == true)){
      int x = Brain.Screen.xPosition();
      
      if ((x<=230)){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
        autonomous_left();
      }else if((x>=250)){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
        autonomous_right();
      }
    }
    else if ((Controller1.ButtonY.pressing()==true)||(Controller1.ButtonA.pressing()==true)){      
      if ((Controller1.ButtonY.pressing()==true)){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
        autonomous_left();
      }else if((Controller1.ButtonA.pressing())){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
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
  //User control code here, inside the loop
    thread rc_auto_loop_controller(rc_auto_loop_function_Controller1);
    wait(20,msec);
}

//
// Main will set up the competition functions and callbacks.
//

int main() {
  // Run the pre-autonomous function.
  pre_auton();

  /* task odomTask(odomTrack);   */

  run();
  
  Brain.Screen.setFillColor(black);

 // Set up callbacks for autonomous and driver control periods based on button presses on the run screen.
  if (autonchoice == 0){
    drawLogo();
    Competition.autonomous(autonomous_left);
    Competition.drivercontrol(user);
  } else if (autonchoice == 1){
    drawLogo();
    Competition.autonomous(autonomous_right);
    Competition.drivercontrol(user);
  } else if (autonchoice == 2){
    drawLogo();
    user();
  } else if(autonchoice == 3){
    autonomous_start();
  }

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(30, msec);
  }
}
