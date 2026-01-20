#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "v5.h"
#include "v5_vcs.h"

#include "robot-config.h"

extern brain Brain;
extern controller Controller1;
extern smartdrive Drivetrain;
extern inertial DrivetrainInertial;
extern motor_group LeftDriveSmart;
extern motor_group RightDriveSmart;
extern motor intakeMotor;
extern motor intakeMotor2;
extern digital_out matchLoad;
extern digital_out middleGoal;
extern digital_out descore;
extern motor_group allIntake;
extern rotation YTracking;

extern inertial DrivetrainInertial;
extern bool running;
extern int autonchoice;

#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)