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

#define COUNTERCLOCKWISE(x) x
#define CLOCKWISE(x) -x
#define ROTATION_TOLERANCE_DEGREES 5
#define ROTATION_SPEED 0.5

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

  isTurning = false;
}

void DriveTrain::Execute() {
  if (isTurning) {
    return;
  }
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


bool DriveTrain::TurnToAngle(double degrees) {
  double currentAngle = fmod(gyro->GetAngle(), 360) - 180;
  double targetAngle = fmod(gyro->GetAngle(), 360) - 180;

  double diff = currentAngle - targetAngle;

  if (fabs(diff) < ROTATION_TOLERANCE_DEGREES) {
    isTurning = false;
    driveTrain->TankDrive(0.0, 0.0);
  } else if (diff > 0) {
    // Turn right
    driveTrain->TankDrive(-ROTATION_SPEED, ROTATION_SPEED);
    isTurning = true;
  } else {
    // Turn left
    driveTrain->TankDrive(ROTATION_SPEED, -ROTATION_SPEED);
    isTurning = true;
  }
  
  return !isTurning;
}

// Forward or backward is determined by inches
// Speed should always be positive
bool DriveTrain::DriveStraight(double inches, double speed) {
  if (!isDrivingStraight) {
    startingLeftEncoder = leftDriveEncoder->Get();
    isDrivingStraight = true;
  }

  if (inches > 0) {
    if (DRIVE_DISTANCE_INCHES(leftDriveEncoder->Get() - startingLeftEncoder) > inches) {
      isDrivingStraight = false;
    }
    speed = DRIVE_FORWARD_SPEED(speed);
  } else {
    if (DRIVE_DISTANCE_INCHES(leftDriveEncoder->Get() - startingLeftEncoder) < inches) {
      isDrivingStraight = false;
    }

    speed = DRIVE_BACKWARD_SPEED(speed);
  }

  return !isDrivingStraight;
}
