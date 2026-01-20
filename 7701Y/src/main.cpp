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
motor leftMotorA = motor(PORT15, ratio6_1, true);
motor leftMotorB = motor(PORT14, ratio6_1, true);
motor leftMotorC = motor(PORT13, ratio6_1, true);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB, leftMotorC);
motor rightMotorA = motor(PORT7, ratio6_1, false);
motor rightMotorB = motor(PORT5, ratio6_1, false);
motor rightMotorC = motor(PORT6, ratio6_1, false);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB, rightMotorC);
inertial DrivetrainInertial = inertial(PORT3);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 259.34, 289.56, 254, mm, 1.3333333333);
motor intakeMotor = motor(PORT10, ratio6_1, false);
motor intakeMotor2 = motor(PORT8, ratio18_1, true);
motor_group allIntake = motor_group(intakeMotor, intakeMotor2);
digital_out matchLoad = digital_out(Brain.ThreeWirePort.C);
digital_out middleGoal = digital_out(Brain.ThreeWirePort.B);
digital_out descore = digital_out(Brain.ThreeWirePort.H);
distance distanceSensor = distance(PORT4);

rotation YTracking = rotation(PORT12, true);




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
  descore.set(true);
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

void user(void) {
  //User control code here, inside the loop
    thread rc_auto_loop_controller(rc_auto_loop_function_Controller1);
    thread controller(controllerUpdating);
    wait(20,msec);
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


void autonomous_skills(void){
  DrivetrainInertial.setRotation(90, degrees);
  moveFor(51, 10);
  turn_to_angle(180, false);
  matchLoad.set(true);
  intakeMotor.spin(reverse, 100, percent);
  wait(200, msec);
  smoothDrive(86, 700);
  Drivetrain.stop();
  wait(20, msec);
  jitter(2750);
  smoothDrive(-70, 500);
  Drivetrain.stop();
  turn_to_angle(270, false);
  while(distanceSensor.objectDistance(mm)>250){
    RightDriveSmart.spin(reverse, 20, percent);
    LeftDriveSmart.spin(reverse, 20, percent);
  }
  intakeMotor.stop();
  DrivetrainInertial.setRotation(0, degrees);
  matchLoad.set(false);
  wait(200, msec);
  turn_to_angle(90, false);
  moveFor(93, 15);
  turn_to_angle(0, false);
  moveFor(9, 6);
  turn_to_angle(88, false);
  smoothDrive(-85, 800);
  intakeMotor2.spin(reverse, 100, percent);
  intakeMotor.spin(reverse, 100, percent);
  DrivetrainInertial.resetRotation();
  wait(2000, msec);
  matchLoad.set(true);
  intakeMotor2.stop();
  smoothDrive(86, 700);
  jitter(2500);
  smoothDrive(-80, 1100);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2000 , msec);
  matchLoad.set(false);
  wait(100, msec);
  allIntake.stop();
  smoothDrive(70, 500);
  turn_to_angle(-90, false);
  moveFor(103,10);
  turn_to_angle(0, false);
  intakeMotor.spin(reverse, 100, percent);
  matchLoad.set(true);
  wait(300, msec);
  smoothDrive(86, 1100);
  jitter(2500);
  smoothDrive(-80, 1100);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2000, msec);
  allIntake.stop();
  Drivetrain.stop();
  wait(100, msec);
  matchLoad.set(false);
  DrivetrainInertial.resetRotation();
  moveFor(14, 3);
  turn_to_angle(90, false);
  while(distanceSensor.objectDistance(mm)>250){
    RightDriveSmart.spin(reverse, 20, percent);
    LeftDriveSmart.spin(reverse, 20, percent);
  }
  turn_to_angle(180, false);
  moveFor(93, 15);
  wait(100, msec);
  DrivetrainInertial.resetRotation();
  turn_to_angle(-90, false);
  moveFor(9,6);
  turn_to_angle(0, false);
  intakeMotor.spin(reverse, 100, percent);
  matchLoad.set(true);
  smoothDrive(86, 700);
  jitter(2500);
  smoothDrive(-80, 1100);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2500, msec);
  allIntake.stop();
  Drivetrain.stop();
}


// Code for left side autonomous (basic)
void autonomous_left(void){
  turn_to_angle(-20, false);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 10, percent);
  moveFor(24, 18);
  turn_to_angle(0, false);
  while(distanceSensor.objectDistance(mm)>650){
    RightDriveSmart.spin(reverse, 50, percent);
    LeftDriveSmart.spin(reverse, 50, percent);
  }
  Drivetrain.stop();
  turn_to_angle(-90, false);
  moveFor(24, 5);
  turn_to_angle(180, false);
  matchLoad.set(true);
  wait(50, msec);
  smoothDrive(90, 700);
  wait(800, msec);
  Drivetrain.stop();
  wait(20, msec);
  wait(300, msec);
  Drivetrain.stop();
  wait(20, msec);
  smoothDrive(-80, 1000);
  wait(10, msec);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2000 , msec);
  allIntake.stop();
  Drivetrain.stop();
}

void autonomous_right_AWP(void){
  turn_to_angle(20, false);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 10, percent);
  moveFor(22, 18);
  turn_to_angle(-45, false);
  moveFor(13, 5);
  intakeMotor.spin(forward, 75, percent);
  wait(1000, msec);
  allIntake.stop();
  wait(100, msec);
  intakeMotor.spin(reverse, 100, percent);
  smoothDrive(-76, 300);
  wait(30, msec);
  turn_to_angle(135, false);
  moveFor(40, 5);
  turn_to_angle(180, false);
  matchLoad.set(true);
  wait(50, msec);
  smoothDrive(90, 700);
  Drivetrain.stop();
  jitter(1000);
  wait(320, msec);
  Drivetrain.stop();
  wait(20, msec);
  smoothDrive(-80, 1100);
  wait(10, msec);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2000 , msec);
  allIntake.stop();
  Drivetrain.stop();
}

// Code for right side autonomous (basic)
void autonomous_right(void) {
  turn_to_angle(20, false);
  intakeMotor.spin(reverse, 100, percent);
  intakeMotor2.spin(forward, 10, percent);
  moveFor(24, 18);
  turn_to_angle(0, false);
  while(distanceSensor.objectDistance(mm)>650){
    RightDriveSmart.spin(reverse, 50, percent);
    LeftDriveSmart.spin(reverse, 50, percent);
  }
  Drivetrain.stop();
  turn_to_angle(90, false);
  moveFor(24, 5);
  turn_to_angle(180, false);
  matchLoad.set(true);
  wait(50, msec);
  smoothDrive(90, 700);
  jitter(1500);
  Drivetrain.stop();
  wait(20, msec);
  wait(300, msec);
  Drivetrain.stop();
  wait(20, msec);
  smoothDrive(-80, 1000);
  wait(10, msec);
  intakeMotor2.spin(reverse, 100, percent);
  wait(2000 , msec);
  allIntake.stop();
  Drivetrain.stop();
}

void autonomous_blank( void ){
  allIntake.spin(reverse, 100, percent);
  Drivetrain.setDriveVelocity(100, percent);
  Drivetrain.drive(forward);
  wait(2200, msec);
  jitter(12000);
}


void autonomous_start( void ){
  Controller1.Screen.clearScreen();
  Brain.Screen.drawImageFromFile("LeftAuto.png", 0, 8);

  Brain.Screen.drawImageFromFile("RightAuto.png", 250, 8);

  Brain.Screen.drawImageFromFile("SKILLS.png", 0, 130);

  Brain.Screen.drawImageFromFile("WP.png", 250, 130);


  Controller1.Screen.setCursor(0,0);
  Controller1.Screen.print("      X: SKILLS        ");
  Controller1.Screen.newLine();
  Controller1.Screen.print("Y: LEFT       A: RIGHT");
  Controller1.Screen.newLine();
  Controller1.Screen.print("     B: RIGHT AWP     ");


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
      }else if((y>130)&&(x<230)){
        practice = true;

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("SKILLSpressed.png", 5, 140);
        }

        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        wait(2000, msec);
        autonomous_blank();
      }else if((y>130)&&(x>=250)){
                practice = true;

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("WPpressed.png", 260, 140);
        }

        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        wait(2000, msec);
        autonomous_right_AWP();
      }
    }
    else if ((Controller1.ButtonY.pressing()==true)||(Controller1.ButtonA.pressing()==true)||(Controller1.ButtonB.pressing()==true)||(Controller1.ButtonX.pressing()==true)){      
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
        wait(1000, msec);
        autonomous_right_AWP();
      }else if((Controller1.ButtonX.pressing())){
        practice = true;
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        wait(1000, msec);
        autonomous_skills();
      }
    }
  }
}

void rightSideStart(){
  Brain.Screen.clearScreen();
  Controller1.Screen.clearScreen();

  Brain.Screen.drawImageFromFile("RightAuto.png", 0, 8);
  Brain.Screen.drawImageFromFile("WP.png", 250, 8);
  


  Controller1.Screen.setCursor(0,0);
  Controller1.Screen.print("                      ");
  Controller1.Screen.newLine();
  Controller1.Screen.print("Y: REGULAR     A: AWP");
  Controller1.Screen.newLine();
  Controller1.Screen.print("                      ");

  while (true){
    if (Brain.Screen.pressing()==true){
      double x = Brain.Screen.xPosition();
      double y = Brain.Screen.yPosition();

      if ((x<=230)&&(y<=130)){

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("RightAutoPressed.png", 5, 18);
        }
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        Competition.autonomous(autonomous_right);
        Competition.drivercontrol(user);

      }else if((x>=250)&&(y<=130)){

        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("WPpressed.png", 260, 18);
        }
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        Competition.autonomous(autonomous_right_AWP);
        Competition.drivercontrol(user);

      }

    }else if((Controller1.ButtonY.pressing() == true)||(Controller1.ButtonA.pressing()==true)){
      if(Controller1.ButtonA.pressing() == true){
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        Competition.autonomous(autonomous_right_AWP);
        Competition.drivercontrol(user);
      }else if(Controller1.ButtonY.pressing()==true){
        Brain.Screen.clearScreen();
        Controller1.Screen.clearScreen();
        drawLogo();
        Competition.autonomous(autonomous_right);
        Competition.drivercontrol(user);
      }
    }
  }

}

//
// Main will set up the competition functions and callbacks.
//

int main() {
  // Run the pre-autonomous function.
  pre_auton();

  run();
  
  Brain.Screen.setFillColor(black);

 // Set up callbacks for autonomous and driver control periods based on button presses on the run screen.
  if (autonchoice == 0){
    drawLogo();
    Competition.autonomous(autonomous_left);
    Competition.drivercontrol(user);
  } else if (autonchoice == 1){
    rightSideStart();
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