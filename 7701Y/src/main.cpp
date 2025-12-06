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
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 259.34, 289.56, 254, mm, 1.3333333333);
motor intakeMotor = motor(PORT5, ratio6_1, false);
motor intakeMotor2 = motor(PORT4, ratio18_1, false);
motor_group allIntake = motor_group(intakeMotor, intakeMotor2);
digital_out sol1 = digital_out(Brain.ThreeWirePort.H);
digital_out sol2 = digital_out(Brain.ThreeWirePort.G);


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
  thread BrainLoading(loadingScreen);
  // calibrate the drivetrain Inertial
  wait(200, msec);
  DrivetrainInertial.calibrate();
  // wait for the Inertial calibration process to finish
  while (DrivetrainInertial.isCalibrating()) {
    wait(25, msec);
  }
  wait(250, msec);
  // reset the screen now that the calibration is complete
  BrainLoading.interrupt();
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);
  running = false;
}

void pid_test(){
  turn_to_angle(135, true);
  Brain.Screen.clearScreen();
}
void autonomous_skills(void){
  smoothDrive(85, 850);
  Drivetrain.stop();
  wait(30, msec);
  turn_to_angle(90, false);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  smoothDrive(55, 1700);     
  Drivetrain.stop(); 
  wait(25, msec);    
  turn_to_angle(140, false);
  smoothDrive(90, 1250);
  Drivetrain.stop();
  turn_to_angle(180, false);
  smoothDrive(-75,1050);
  allIntake.spin(reverse, 100, percent);
  wait(1500, msec);
  Drivetrain.stop();
  allIntake.stop();
  sol1.set(true);
  smoothDrive(40, 300);
  Drivetrain.stop();
  turn_to_angle(180, false);
  smoothDrive(93, 1200);
  Drivetrain.stop();
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  jitter(3500);
  smoothDrive(-55, 300);
  Drivetrain.stop();
  turn_to_angle(180, false);
  smoothDrive(-100, 1250);
  allIntake.spin(reverse, 100, percent);
  wait(1100, msec);
  Drivetrain.stop();
  allIntake.stop();
  allIntake.spin(forward, 30, percent);
  wait(300, msec);
  allIntake.stop();
  smoothDrive(75, 700);
  Drivetrain.stop();
  sol1.set(false);
  wait(25, msec);
  turn_to_angle(315, false);
  smoothDrive(90, 950);
  Drivetrain.stop();
  wait(25, msec);
  turn_to_angle(270, false);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  smoothDrive(90, 2100);
  Drivetrain.stop();
  /*wait(25, msec);
  turn_to_angle(-135, false);
  smoothDrive(90, 1400);
  Drivetrain.stop();
  wait(25, msec);
  turn_to_angle(-180, false);
  smoothDrive(-75,1050);
  allIntake.spin(reverse, 100, percent);
  wait(1500, msec);
  Drivetrain.stop();
  allIntake.stop();
  sol1.set(true);
  smoothDrive(40, 300);
  Drivetrain.stop();
  turn_to_angle(-180, false);
  smoothDrive(85, 1000);
  Drivetrain.stop();
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  jitter(3000);
  smoothDrive(-100, 1250);
  allIntake.spin(reverse, 100, percent);
  wait(1500, msec);
  Drivetrain.stop();
  allIntake.stop();*/
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
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  smoothDrive(80, 1200);
  Drivetrain.stop();
  wait(20, msec);
  Drivetrain.stop();
  wait(200, msec);
  Drivetrain.stop();
  turn_to_angle(-124, false);
  allIntake.stop();
  smoothDrive(95, 1260);
  wait(100, msec);
  turn_to_angle(-165, false);
  smoothDrive(-75, 1050);
  allIntake.spin(reverse, 100, percent);
  wait(1400 ,msec);
  Drivetrain.stop();
  sol1.set(true);
  allIntake.stop();
  smoothDrive(50, 400);
  turn_to_angle(-165, false);
  smoothDrive(85, 1500);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 5, percent);
  jitter(1800);
  Drivetrain.stop();
  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.drive(reverse);
  wait(1000, msec);
  allIntake.spin(reverse, 90, percent);
  Drivetrain.stop();
}

// Code for right side autonomous (basic)
void autonomous_right(void) {
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 12, percent);
  smoothDrive(80, 1200);
  Drivetrain.stop();
  wait(20, msec);
  Drivetrain.stop();
  wait(200, msec);
  Drivetrain.stop();
  turn_to_angle(124, false);
  allIntake.stop();
  smoothDrive(95, 1260);
  wait(100, msec);
  turn_to_angle(165, false);
  smoothDrive(-75, 1050);
  allIntake.spin(reverse, 100, percent);
  wait(1400 ,msec);
  Drivetrain.stop();
  sol1.set(true);
  allIntake.stop();
  smoothDrive(50, 400);
  turn_to_angle(165, false);
  smoothDrive(85, 1500);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 5, percent);
  jitter(1800);
  Drivetrain.stop();
  Drivetrain.setDriveVelocity(30, percent);
  Drivetrain.drive(reverse);
  wait(450, msec);
  Drivetrain.stop();
  turn_to_angle(165, false);
  Drivetrain.setDriveVelocity(90, percent);
  Drivetrain.drive(reverse);
  wait(1000, msec);
  allIntake.spin(reverse, 90, percent);
  Drivetrain.stop();
}

void autonomous_start( void ){
  Controller1.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("LeftAuto.png", 0, 8);

  Brain.Screen.drawImageFromFile("PIDTest.png", 0, 130);

  Brain.Screen.drawImageFromFile("RightAuto.png", 250, 8);
  Controller1.Screen.setCursor(0,0);
  Controller1.Screen.print("                       ");
  Controller1.Screen.newLine();
  Controller1.Screen.print("Y: LEFT       A: RIGHT");
  Controller1.Screen.newLine();
  Controller1.Screen.print("      B: PID TEST      ");


  while(true){
    if ((Brain.Screen.pressing() == true)){
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();
      
      if ((x<=230)&&(y<=130)){
        practice = true;

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("LeftAutoPressed.png", 5, 18);
        }

        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
        autonomous_left();
      }else if((x>=250)&&(y<=130)){
        practice = true;

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("RightAutoPressed.png", 260, 18);
        }

        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        wait(1000, msec);
        autonomous_right();
      }else if((y>130)){
        practice = true;

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("PIDTestPressed.png", 5, 140);
        }

        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        wait(2000, msec);
        autonomous_skills();
      }
    }
    else if ((Controller1.ButtonY.pressing()==true)||(Controller1.ButtonA.pressing()==true)||(Controller1.ButtonB.pressing()==true)){      
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
      }else if((Controller1.ButtonB.pressing())){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        thread BrainScreenUpdate(BrainScreenUpdate);
        wait(1000, msec);
        autonomous_skills();
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
    thread controller(controllerUpdating);
    wait(20,msec);
}

//
// Main will set up the competition functions and callbacks.
//

int main() {
  // Run the pre-autonomous function.
  pre_auton();

  // thread odomTask(odomTrack); 

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
