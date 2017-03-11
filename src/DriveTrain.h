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
  DriveTrain(DriverControls *controls);
  void Initialize();
  void Execute();
  void Log();

  bool TurnToAngle(double degrees);
  bool DriveStraight(double inches, double speed=0.5);

  void LockShifterInGear(ShifterGear gear);
  void UnlockShifterGear();
  
  virtual ~DriveTrain();

 private:
  RobotDrive *driveTrain;
  DoubleSolenoid *shifter;
  
  DriverControls *controls;
  
  Encoder *leftDriveEncoder;
  Encoder *rightDriveEncoder;

  ADXRS450_Gyro *gyro;

  ShifterGear currentGear;
  void RunShifter();

  bool isTurning;
  bool isDrivingStraight;
  double targetAngle;
  double currentAngle;
  int startingLeftEncoder;

  bool isShifterLocked;

  double leftSpeed, rightSpeed;
  double leftEncoderValue, rightEncoderValue;
};

#endif /* SRC_DRIVETRAIN_H_ */
