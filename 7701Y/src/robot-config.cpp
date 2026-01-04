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
bool intakesNeedToBeStopped = false;
bool matchLoadToggleLast =false;
bool matchLoadToggle = true;
bool middleGoalToggleLast =false;
bool middleGoalToggle = false;
bool descoreToggleLast =false;
bool descoreToggle = false;

//These booleans are used so only one button has to be pressed to do an action
//This is because button presses are detected many times in a second, so it would just go on and off without bound.

float Kpp = 0.355;   // Proportional gain for PID
float Ki = 0.0;   // Integral gain for PID
float Kd = 0.0;   // Derivative gain for PID
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
  Drivetrain.setDriveVelocity(100, percent);
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
      
      int deadzone = 2;

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
        double leftVelocity = (drivetrainLeftSideSpeed*fabs(drivetrainLeftSideSpeed))/115;

        LeftDriveSmart.setVelocity(leftVelocity, percent);
        LeftDriveSmart.spin(forward);
      }

      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        double rightVelocity = (drivetrainRightSideSpeed*fabs(drivetrainRightSideSpeed))/115;

        RightDriveSmart.setVelocity(rightVelocity, percent);
        RightDriveSmart.spin(forward);
      }

      // This section is for button holds and toggles for other actions on the robot (solenoids, intake, etc)
      

      if ((Controller1.ButtonB.pressing() == true)){
        if ( matchLoadToggleLast == false){
          if (matchLoadToggle == true){
            matchLoad.set(true); 
            matchLoadToggle = false;
          }
          else if (matchLoadToggle == false)
          {
            matchLoad.set(false);
            matchLoadToggle = true;
          } 
           matchLoadToggleLast = true;
        }
      }
      if (Controller1.ButtonB.pressing() == false){
        matchLoadToggleLast = false;
      }

      if ((Controller1.ButtonY.pressing() == true)){
        if (middleGoalToggleLast == false){
          if (middleGoalToggle == false){
            middleGoal.set(true); 
            middleGoalToggle = true;
          }
          else if (middleGoalToggle == true)
          {
            middleGoal.set(false);
            middleGoalToggle = false;
          } 
          middleGoalToggleLast = true;
        }
      }
      if (Controller1.ButtonY.pressing() == false){
        middleGoalToggleLast = false;
      }


      if ((Controller1.ButtonA.pressing() == true)){
        if (descoreToggleLast == false){
          if (descoreToggle == true){
            descore.set(true); 
            descoreToggle = false;
          }
          else if (descoreToggle == false)
          {
            descore.set(false);
            descoreToggle = true;
          } 
          descoreToggleLast = true;
        }
      }
      if (Controller1.ButtonA.pressing() == false){
        descoreToggleLast = false;
      }



      if ((Controller1.ButtonR1.pressing() == true)&&(intakesNeedToBeStopped == false)){
          intakeMotor.spin(reverse, 100, percent);
          intakeMotor2.spin(reverse, 100, percent);
          intakesNeedToBeStopped=true;
      }
      if ((Controller1.ButtonR2.pressing() == true)&&(intakesNeedToBeStopped == false)){
          intakeMotor.spin(forward, 100, percent);
          intakeMotor2.spin(forward, 100, percent);
          intakesNeedToBeStopped=true;
        }
      if ((Controller1.ButtonL1.pressing() == true)&&(intakesNeedToBeStopped == false)){
          intakeMotor.spin(reverse, 100, percent);
          intakeMotor2.spin(forward, 12, percent);
          intakesNeedToBeStopped=true;
      }
      if ((Controller1.ButtonL2.pressing() == true)&&(intakesNeedToBeStopped == false)){
          intakeMotor.spin(forward, 100, percent);
          intakesNeedToBeStopped=true;
        }
      if ((intakesNeedToBeStopped==true)&&(Controller1.ButtonR1.pressing() == false)&&(Controller1.ButtonR2.pressing()==false)&&(Controller1.ButtonL1.pressing() == false)&&(Controller1.ButtonL2.pressing()==false)){
          intakeMotor.stop();
          intakeMotor2.stop();
          intakesNeedToBeStopped = false;
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

void smoothDrive(int targetSpeed, int duration_ms) {
  double k = 0.015;          // reduced steepness
  int steps = duration_ms / 30;
  double midpoint = steps / 2.0;

  for (int i = 0; i <= steps; i++) {
    double progress = 1.0 - (1.0 / (1.0 + exp(-k * (i - midpoint))));
    int currentSpeed = progress * targetSpeed;

    Drivetrain.setDriveVelocity(currentSpeed, percent);
    Drivetrain.drive(forward);
    wait(30, msec);
  }
}

void turn_to_angle(float targetAngle, bool test) {
    float error;            // Difference between target and current angle
    float previousError = 0;
    float integral = 0;
    float derivative;
    float motorPower;
    int maxIterations = 500;


    std::string data_buffer = "time, drive-angle ,error\n";
    int iterations = 0;

    if(test == true){
      if(!Brain.SDcard.isInserted()){
        Brain.Screen.setCursor(1,1);
        Brain.Screen.print("SD card is missing!");
        while(true){
          wait(5, msec);
        }
      }
      Brain.Timer.clear();
    }


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

        if(test==true){
          char line[128];
          sprintf(line, "%.3f, %.3f\n", Brain.Timer.value(), driveangle);
          data_buffer += line;
        }

        if(iterations<maxIterations){
          iterations++;
        }else{
          break;
        }

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
    wait(50, msec);
    if(test == true){
      Brain.Screen.setCursor(4,1);
      bool writeOK = Brain.SDcard.savefile("pidtest.csv", (uint8_t*)data_buffer.c_str(), data_buffer.length());

      if(!writeOK){
        Brain.Screen.print("SD card WRITE error");
      }else{
        Brain.Screen.print("SD card written");
      }
    }
}
/*------------------------------------------------*/
/*                                                */
/*               ODOMETRY SYSTEM                  */
/*                    ||||                        */
/*                                                */
/*---------------------\/-------------------------*/


double xPos = 0.0;
double yPos = 0.0;
double headingRad = 0.0;

const double wheelRadius = 1; // inches (2" diameter / 2)
const double sideWheelOffset = 1.5; // distance from robot center to the sideways wheel
const double forwardWheelOffset = 3.0; // distance from center to forward wheel
const double degToRad = 3.14159265 / 180.0;

double degToInches(double deg) {
  return (deg * 3.14159265 / 180.0) * wheelRadius;
}

template <typename T>
T clamp(T value, T minVal, T maxVal) {
  if (value < minVal) return minVal;
  if (value > maxVal) return maxVal;
  return value;
}

int odomTrack() {
  double prevX = 0.0, prevY = 0.0;
  double prevHeading = DrivetrainInertial.rotation();

  while (true) {
    // Read sensors
    double currX = degToInches(XTracking.position(degrees));
    double currY = degToInches(YTracking.position(degrees));
    double headingDeg = DrivetrainInertial.rotation();
    double headingChange = (headingDeg - prevHeading) * degToRad;

    double dX = currX - prevX;
    double dY = currY - prevY;

    prevX = currX;
    prevY = currY;
    prevHeading = headingDeg;

    headingRad = headingDeg * degToRad;

    // Local motion correction (due to rotation)
    double localX = dX - (headingChange * sideWheelOffset);
    double localY = dY + (headingChange * forwardWheelOffset);

    // Convert local to global coordinates
    xPos += localX * cos(headingRad) - localY * sin(headingRad);
    yPos += localX * sin(headingRad) + localY * cos(headingRad);

    wait(10, msec);
  }
  return 0;
}

// Move to a specific coordinate
void moveFor(double targetD) {
  double speed = 20;
  double slowDownRadius= 10.0;
  double Pos =0.0;
  const double tolerance = 0.5; // inches
  YTracking.resetPosition();
  while (fabs(targetD-Pos) > tolerance) {
    Pos = degToInches(YTracking.position(degrees));

    double delta = targetD - Pos;
    LeftDriveSmart.spin(fwd, speed, pct);
    RightDriveSmart.spin(fwd, speed, pct);

    wait(20, msec);
  }

  LeftDriveSmart.stop();
  RightDriveSmart.stop();
}

/*----------------------/\------------------------*/
/*                                                */
/*               ODOMETRY SYSTEM                  */
/*                    ||||                        */
/*                                                */
/*------------------------------------------------*/

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
    Brain.Screen.drawImageFromFile("LeftAuto.png", 0 , 8);
    Brain.Screen.drawImageFromFile("RightAuto.png",250, 8);
    Brain.Screen.drawImageFromFile("UserCode.png", 0,130);
    Brain.Screen.drawImageFromFile("AutoPractice.png", 250, 130);
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
      break;
    }
    if(Controller1.ButtonY.pressing() == true){
      autonchoice = 0;
      break;
    }
    if(Controller1.ButtonX.pressing() == true){
      autonchoice = 1;
      break;
    }
    if(Controller1.ButtonA.pressing() == true){
      autonchoice = 3;
      break;
    }

    if (Brain.Screen.pressing()){
      int x = Brain.Screen.xPosition();
      int y = Brain.Screen.yPosition();

      if ((x<=230)&&(y<=130)){
        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("LeftAutoPressed.png", 10, 18);
        }
        autonchoice = 0;
        break;
      }else if((y<=130)&&(x>=250)){
        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("RightAutoPressed.png", 260, 18);
        }
        autonchoice = 1;
        break;
      }else if((y>130)&&(x<230)){
        Brain.Screen.clearScreen();
        while (Brain.Screen.pressing())
        {
          Brain.Screen.drawImageFromFile("UserCodePressed.png", 10, 140);
        }
        
        autonchoice = 2;
        break;
      }else if((y>130)&&(x>250)){
        Brain.Screen.clearScreen();
        while(Brain.Screen.pressing()){
          Brain.Screen.drawImageFromFile("AutoPracticePressed.png", 260, 140);
        }
        autonchoice = 3;
        break;
      }
    }
    wait(10, msec);
  }
  Brain.Screen.clearScreen();
  Controller1.Screen.clearScreen();
  wait(150, msec);
}
void jitter(int dmsec){
  int steps = dmsec / 400;
  for(int i =0; i<=steps ;i++){
    Drivetrain.setDriveVelocity(55, percent);
    Drivetrain.drive(forward);
    wait(250, msec);
    Drivetrain.drive(reverse);
    wait(150, msec);
    Drivetrain.stop();
  }
}


void drawLogo() {
  Brain.Screen.drawImageFromFile("ragebait.png", 0,0);
}

int loadingScreen(){
  while(1){
    Brain.Screen.drawImageFromFile("logoscreen1.png",25,0);
    wait(160, msec);

    Brain.Screen.drawImageFromFile("logoscreen2.png",25,0);
    wait(160, msec);

    Brain.Screen.drawImageFromFile("logoscreen3.png",25,0);
    wait(160, msec);

    Brain.Screen.drawImageFromFile("logoscreen4.png",25,0);
    wait(160, msec);

    Brain.Screen.drawImageFromFile("logoscreen5.png",25,0);
    wait(160, msec);

  }
  return 0;
}

int controllerUpdating(){
  std::string back;
  std::string matchload;
  while(1){
    if(descoreToggle == false){
      back = "false";
    }else{
      back = "true";
    }
    if(matchLoadToggle == true){
      matchload = "false";
    }else{
      matchload = "true";
    }
    if(middleGoalToggle == true){
      Controller1.rumble("..- ..- ..- ..- ..- ");
    }else{
      Controller1.rumble(" ");
    }
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print("HOOK?: %s", back.c_str());
    Controller1.Screen.newLine();
    Controller1.Screen.print("MATCHLOAD?: %s", matchload.c_str());
    wait(50, msec);
    Controller1.Screen.clearScreen();
  }
  return 0;
}