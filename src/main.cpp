/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\coulterjus                                       */
/*    Created:      Thu Dec 05 2019                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

#define LEFT -1
#define RIGHT 1
#define FORWARD 1
#define REVERSE -1

#define WHEEL_DIAM 3.25
#define DRIVE_GEAR_RATIO 5/3

#define M_PI 3.14159265358979323846

double armPos = -1;
int selectedAutoMode = 0;

using namespace vex;

vex::brain       Brain;
vex::competition Competition;

vex::motor FLDrive(PORT10, gearSetting::ratio6_1, false);
vex::motor BLDrive(PORT9, gearSetting::ratio6_1, false);
vex::motor FRDrive(PORT1, gearSetting::ratio6_1, true);
vex::motor BRDrive(PORT2, gearSetting::ratio6_1, true);

////Intake Motors
vex::motor LeftIntake(PORT8,gearSetting::ratio36_1,false);
vex::motor RightIntake(PORT3,gearSetting::ratio36_1,true);

////Lift Motor
vex::motor LiftMotor(PORT7,gearSetting::ratio36_1,true);
vex::motor TiltMotor(PORT4,gearSetting::ratio36_1,true);

vex::motor_group LDrives(FLDrive, BLDrive);
vex::motor_group RDrives(FRDrive, BRDrive);
vex::motor_group ADrives(FLDrive, FRDrive, BLDrive, BRDrive);
vex::motor_group Intakes(LeftIntake, RightIntake);

vex::inertial Inertial(PORT6);

vex::controller Controller1;

/**
 * @brief Used to resign bits
 * 
 * @param x the bit
 * @return the resigned bit
 */
int reSign(int x) {
  return signbit(x) ? -1 : 1;
}

/**
 * @brief Accelerates a motor with the provided parameters.
 * 
 * @param minSpeed the minimum speed to spin at
 * @param maxSpeed the maximum speed to spin at
 * @param startPos the starting position of the motor
 * @param endPos   the ending position of the motor
 * @param moto     the motor that should be spinning
 * @param dir      the direction in which to accelerate
 */
void accelByDist(double minSpeed, double maxSpeed, double startPos, double endPos, vex::motor moto, vex::directionType dir) {
  double posDiff = endPos - startPos;
  double currentPos = moto.position(vex::rotationUnits::deg) - startPos;
  double speed = fabs((minSpeed - maxSpeed) * (1 - (currentPos / posDiff)));
  double vel = (speed + minSpeed);

  moto.spin(dir, (int) vel, vex::velocityUnits::pct);
}

/**
 * @brief Acceleration code intended for autonomous driving.
 * 
 * @param minSpeed the minumum speed to spin at
 * @param maxSpeed the maximum speed to spin at
 * @param startPos the starting position of the motor
 * @param endPos   the ending position of the motor
 * @param moto     the motor that should be spinning
 * @param dir      the direction in which to accelerate
 */
void autonAccelByDist(double minSpeed, double maxSpeed, double startPos, double endPos, vex::motor moto, vex::directionType dir) {
  int endPosCorrected = (endPos > startPos) ? endPos - 5 : endPos + 5;
  
  while (moto.position(vex::rotationUnits::deg) < endPosCorrected) {
    accelByDist(minSpeed, maxSpeed, startPos, endPos, moto, dir);
  }

  moto.stop();
}

/**
 * @brief The main function for autonomous operation.
 * 
 * @param minSpeed   the minumum speed to spin at
 * @param maxSpeed   the maximum speed to spin at
 * @param distanceIn the distance the target is 
 * @param curve      [placeholder]
 * @param dir        [placeholder]
 */
void autonDrive(double minSpeed, double maxSpeed, double distanceIn, double curve, int dir) {
  ADrives.resetRotation();

  double revs = (distanceIn / (M_PI * WHEEL_DIAM)) * DRIVE_GEAR_RATIO;
  double halfRevs = revs/2;

  while (fabs(ADrives.rotation(vex::rotationUnits::rev)) < fabs(revs)) {
    double currentPos = ADrives.rotation(vex::rotationUnits::rev) - halfRevs;
    double speed = fabs((minSpeed - maxSpeed) * pow(1 - fabs(currentPos / halfRevs), curve));
    double vel = (speed + minSpeed);

    ADrives.spin(vex::directionType::fwd, (int)(vel * dir), vex::velocityUnits::pct);
  }

  ADrives.stop();
}

/**
 * @brief Function handling autonomous turning.
 * 
 * @param minSpeed    the minumum speed to spin at
 * @param maxSpeed    the maximum speed to spin at
 * @param rotationDeg the amount in degrees to rotate 
 * @param curve       [placeholder]
 * @param dir         [placeholder]
 */
void autonTurn(double minSpeed, double maxSpeed, double rotationDeg, double curve, int dir) {
  Inertial.resetRotation();

  double halfRot = rotationDeg / 2;
  while (fabs(Inertial.rotation()) < fabs(rotationDeg)) {
    double currentPos = Inertial.rotation() - halfRot;
    double speed = fabs((minSpeed - maxSpeed) * pow(1-fabs(currentPos/halfRot), curve));
    double vel = (speed + minSpeed);

    LDrives.spin(vex::directionType::fwd, (int)(vel*dir), vex::velocityUnits::pct);
    RDrives.spin(vex::directionType::fwd, (int)(-vel*dir), vex::velocityUnits::pct);
  }

  ADrives.stop();
}

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the cortex has been powered on and    */ 
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

/**
 * @brief All activities that occur before the competition starts (clearing encoders, setting servo positions)
 */
void pre_auton( void ) {
  FLDrive.setRotation(0, rotationUnits::rev);
  BLDrive.setRotation(0, rotationUnits::rev);
  FRDrive.setRotation(0, rotationUnits::rev);
  BRDrive.setRotation(0, rotationUnits::rev);

  TiltMotor.setStopping(vex::brakeType::hold);
  Intakes.setStopping(vex::brakeType::hold);
  LiftMotor.setStopping(vex::brakeType::hold);

  ADrives.setStopping(vex::brakeType::brake);
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  /* --------------------------------------------------------------------------- */
  /*                 One-point autonomous [emergencies only]                     */
  /* --------------------------------------------------------------------------- */ 

    if (selectedAutoMode == 1) {
    autonDrive(50,50,12,1,REVERSE);
    autonDrive(50,50,12,1,FORWARD);

    LiftMotor.spinFor(-1, vex::rotationUnits::rev, true);
    LiftMotor.spinFor(1, vex::rotationUnits::rev, true);
  }

  /* --------------------------------------------------------------------------- */
  /*                        Normal five-point autonomous                         */
  /* --------------------------------------------------------------------------- */

  if (selectedAutoMode == 0) {
    autonDrive(10, 15, 8, 2, FORWARD);
  
    LiftMotor.spinFor(-1, vex::rotationUnits::rev, true);
    LiftMotor.spinFor(1,  vex::rotationUnits::rev, true);

    Intakes.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);

    autonDrive(10, 30, 36, 2, FORWARD);

    Intakes.stop(vex::brakeType::hold);

    autonDrive(10, 30, 24, 2, REVERSE);
    autonTurn(10, 30, 135, 2, RIGHT);
    autonDrive(10, 30, 12, 2, FORWARD);
    autonAccelByDist(10, 50, 0, 640, TiltMotor, vex::directionType::fwd);

    ADrives.spinFor(-2,  vex::rotationUnits::rev, 30, vex::velocityUnits::pct, false);
    Intakes.spinFor(-10, vex::rotationUnits::rev, 50, vex::velocityUnits::pct, true);
  }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  while (1) {
    /* Handles smooth acceleration for stick positions */
    int RStick2 = (int)fabs(127 * pow((Controller1.Axis2.value() / 127.0), 2)) * reSign(Controller1.Axis2.value());
    int RStick1 = (int)fabs(127 * pow((Controller1.Axis1.value() / 127.0), 2)) * reSign(Controller1.Axis1.value());

    /* Thermal Regulation */
    if (FLDrive.temperature(percentUnits::pct) > 85 
          || FLDrive.temperature(percentUnits::pct) > 85 
          || FLDrive.temperature(percentUnits::pct) > 85 
          || FLDrive.temperature(percentUnits::pct) > 85) {
      RStick2 *= 0.75;
      RStick1 *= 0.75;
    }

    LDrives.spin(vex::directionType::fwd, (RStick2 + RStick1)/2, vex::velocityUnits::pct);
    RDrives.spin(vex::directionType::fwd, (RStick2 - RStick1)/2, vex::velocityUnits::pct);

    /* Thermal Feedback */
    if (FLDrive.temperature(percentUnits::pct) > 70
        || FLDrive.temperature(percentUnits::pct) > 70
        || FLDrive.temperature(percentUnits::pct) > 70
        || FLDrive.temperature(percentUnits::pct) > 70) {
      Controller1.rumble("... --- ...   ");
    }

    /* X | Arm height (lowest position) */
    if (Controller1.ButtonX.pressing()) {
      armPos = -270;
    }

    /* A | Arm height (middle tower) */
    if (Controller1.ButtonA.pressing()) {
      armPos = -180;
    }

    /* B | Arm height (side towers) */
    if (Controller1.ButtonB.pressing()) {
      armPos = 0;
    }

    /* R1 + R2 | Intake Controls */
    if (Controller1.ButtonR1.pressing()) {
      Intakes.spin(vex::directionType::fwd, 100, vex::velocityUnits::pct);
    } else if (Controller1.ButtonR2.pressing()) {
      Intakes.spin(vex::directionType::rev, 100, vex::velocityUnits::pct);
    } else {
      Intakes.stop(vex::brakeType::hold);
    }

    /* L1 | Spins lift motor in order to lower the lift. */
    /* L2 | Spins lift motor in order to raise the lift. */
    if (Controller1.ButtonL1.pressing()) {
      armPos = -1;
      LiftMotor.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
    } else if (Controller1.ButtonL2.pressing()) {
      armPos = -1;
      LiftMotor.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
    }
    
    /* Automatic lift height management (macros). Must stay coupled to the button control if statements, or it breaks. */
    else if (armPos != -1 && LiftMotor.position(vex::rotationUnits::deg) + 5 < armPos) {
      LiftMotor.spin(vex::directionType::fwd, 75, vex::velocityUnits::pct);
    } else if (armPos != -1 && LiftMotor.position(vex::rotationUnits::deg) - 5 > armPos) {
      LiftMotor.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
    } else {
      LiftMotor.stop(vex::brakeType::hold);
    }

    /* Up   | Raises drop motor */
    /* Down | Lowers drop motor */
    if (Controller1.ButtonUp.pressing() && (TiltMotor.position(vex::rotationUnits::deg) < 760)) {
      accelByDist(25, 50, 0, 760, TiltMotor, vex::directionType::fwd);
    } else if (Controller1.ButtonDown.pressing() && (TiltMotor.position(vex::rotationUnits::deg) > 0)) {
      TiltMotor.spin(vex::directionType::rev, 75, vex::velocityUnits::pct);
    } else {
      TiltMotor.stop(vex::brakeType::hold);
    }

    /* Y | hell if i know what this shit does */
    if (Controller1.ButtonY.pressing()) {
      Intakes.spin(vex::directionType::rev, 50, vex::velocityUnits::pct);
      ADrives.spin(vex::directionType::rev, 30, vex::velocityUnits::pct);
    }

    /* Sleep the main task to prevent wasting resources. */
    vex::task::sleep(20); 
  }
}

void screenEventHandler(int xPos, int yPos) {
  // Brain.Screen.clearScreen();
  // Brain.Screen.setCursor(0, 0);

  if (Brain.Screen.xPosition() >= 20 && Brain.Screen.yPosition() <= 30) {
    selectedAutoMode = 0;
  }

  if (Brain.Screen.yPosition() >= 70 && Brain.Screen.yPosition() <= 80) {
    selectedAutoMode = 1;
  }
}

/**
 * @brief Main block for the program.
 */
int main() {
    vexcodeInit();

    Competition.autonomous(autonomous);
    Competition.drivercontrol(usercontrol);

    /* Run pre-auton function */
    pre_auton();

    /* Prevent application from closing. */
    while(1) {
      if (selectedAutoMode == 0) {
        Brain.Screen.printAt(50, 30, false, "[Active] 5-Point Autonomous");
        Brain.Screen.printAt(50, 80, false, "1-Point Autonomous");
      } else {
        Brain.Screen.printAt(50, 80, false, "5-Point Autonomous");
        Brain.Screen.printAt(50, 80, false, "[Active] 1-Point Autonomous");
      }
      
      if (Brain.Screen.pressing()) {
        screenEventHandler(Brain.Screen.xPosition(), Brain.Screen.yPosition());
      }

      vex::task::sleep(100);
    }    
       
}