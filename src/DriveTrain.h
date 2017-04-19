/*
 * DriveTrain.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_DRIVETRAIN_H_
#define SRC_DRIVETRAIN_H_

#include <WPILib.h>
#include "PeriodicExecutable.h"
#include "DriverControls.h"

class DriveTrain: public PeriodicExecutable {
 public:
  enum DriveMode {
    tank,
    arcade,
  };

  DriveTrain(DriverControls *controls);
  void Initialize();
  void Execute();
  void Log();

  bool TurnByAngle(double degrees);
  bool DriveStraight(double inches, double speed=0.5);
  void SetDriveMode(DriveMode driveMode);

  void LockShifterInGear(ShifterGear gear);
  void UnlockShifterGear();

  ADXRS450_Gyro *gyro;
  
  virtual ~DriveTrain();

 private:
  RobotDrive *driveTrain;
  DoubleSolenoid *shifter;
  
  DriverControls *controls;
  
  Encoder *leftDriveEncoder;
  Encoder *rightDriveEncoder;

  ShifterGear currentGear;
  void RunShifter();

  bool isTurning;
  bool isDrivingStraight;
  double targetAngle;
  double currentAngle;
  int startingLeftEncoder;

  bool isShifterLocked;

  double leftSpeed, rightSpeed;
  int leftEncoderValue, rightEncoderValue;

  DriveMode currentDriveMode;

  DriveMode driveModes[2];

  SendableChooser<DriveMode*> *driveModeChooser = new SendableChooser<DriveMode*>();
};

#endif /* SRC_DRIVETRAIN_H_ */
