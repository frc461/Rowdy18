/*
 * DriveTrain.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "DriveTrain.h"
#include "Ports.h"

#define SHIFTER_LOW DoubleSolenoid::kReverse
#define SHIFTER_HIGH DoubleSolenoid::kForward

#define DRIVE_DISTANCE_INCHES(x) x * 28.647
#define DRIVE_FORWARD_SPEED(x) -x
#define DRIVE_BACKWARD_SPEED(x) x

DriveTrain::DriveTrain(DriverControls *controls) {
  this->controls = controls;

  Victor *frontLeft = new Victor(frontLeftPWM);
  Victor *backLeft = new Victor(backLeftPWM);
  Victor *frontRight = new Victor(frontRightPWM);
  Victor *backRight = new Victor(backRightPWM);
  driveTrain = new RobotDrive(frontLeft, backLeft, frontRight, backRight);

  leftDriveEncoder = new Encoder(leftDriveEncoderA, leftDriveEncoderB);
  rightDriveEncoder = new Encoder(rightDriveEncoderA, rightDriveEncoderB);

  gyro = new ADXRS450_Gyro(SPI::kOnboardCS0);
  
  shifter = new DoubleSolenoid(shifterForwardPCM, shifterReversePCM);
}

void DriveTrain::Execute() {
  driveTrain->TankDrive(controls->GetLeft(), controls->GetRight());
  RunShifter();
}

void DriveTrain::RunShifter() {
  switch (controls->GetGear()) {
  case ShifterGear::kLowGear:
    shifter->Set(SHIFTER_LOW);
    break;
  case ShifterGear::kHighGear:
    shifter->Set(SHIFTER_HIGH);
    break;
  }
}

void DriveTrain::Log() {

}

DriveTrain::~DriveTrain() {
  // TODO Auto-generated destructor stub
}

