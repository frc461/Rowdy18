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

#define DRIVE_DISTANCE_INCHES(x) x * 20.46
#define DRIVE_FORWARD_SPEED(x) -x
#define DRIVE_BACKWARD_SPEED(x) x

#define COUNTERCLOCKWISE(x) x
#define CLOCKWISE(x) -x
#define ROTATION_TOLERANCE_DEGREES 5
#define ROTATION_SPEED 0.65

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

  driveModes[0] = tank;
  driveModes[1] = arcade;
  driveModeChooser->AddDefault("Tank", &driveModes[0]);
  driveModeChooser->AddObject("Arcade", &driveModes[1]);
  SmartDashboard::PutData("Drive mode", driveModeChooser);

  Initialize();
}

void DriveTrain::Initialize() {
  PeriodicExecutable::Initialize();

  isTurning = false;
  isShifterLocked = false;
  isDrivingStraight = false;
  gyro->Reset();
  leftDriveEncoder->Reset();
  rightDriveEncoder->Reset();

  currentDriveMode = DriveMode::tank;

  currentDriveMode = *(driveModeChooser->GetSelected());
}

void DriveTrain::Execute() {
  leftEncoderValue = leftDriveEncoder->Get();
  rightEncoderValue = rightDriveEncoder->Get();

  if (currentDriveMode == DriveMode::tank || isDrivingStraight || isTurning) {
    driveTrain->TankDrive(leftSpeed, rightSpeed);
  } else {
    driveTrain->ArcadeDrive(leftSpeed, rightSpeed);
  }
  RunShifter();

  if (isTurning || isDrivingStraight) {
    return;
  }


  if (currentDriveMode == DriveMode::tank) {
    leftSpeed = controls->GetLeft();
    rightSpeed = controls->GetRight();
  } else {
    leftSpeed = controls->GetThrottle();
    rightSpeed = controls->GetTurn();
  }
}

void DriveTrain::RunShifter() {
  if (!isShifterLocked) {
    currentGear = controls->GetGear();
  }
  switch (currentGear) {
  case ShifterGear::kLowGear:
    shifter->Set(SHIFTER_LOW);
    break;
  case ShifterGear::kHighGear:
    shifter->Set(SHIFTER_HIGH);
    break;
  }
}

void DriveTrain::Log() {
  Logger::Log(logDriveTrain, "Left speed: %lf, Right speed: %lf\n", leftSpeed, rightSpeed);
  Logger::Log(logDriveTrain, "Left encoder: %d, right encoder: %d\n", leftEncoderValue, rightEncoderValue);
  Logger::Log(logDriveTrain, "Angle: %lf\n", gyro->GetAngle());
  SmartDashboard::PutNumber("Left encoder\n", leftEncoderValue);
  SmartDashboard::PutNumber("Right encoder\n", rightEncoderValue);
  Logger::Log(logDriveTrain, "%s\n", currentGear == ShifterGear::kLowGear ? "Low Gear" : "High Gear");
  if (isTurning)
    Logger::Log(logDriveTrain, "Is turning\n");

  if (isShifterLocked) {
    Logger::Log(logDriveTrain, "Shifter locked\n");
  }

  if (isDrivingStraight) {
    Logger::Log(logDriveTrain, "Is driving straight, started at %lf\n", startingLeftEncoder);
    Logger::Log(logDriveTrain, "Current angle: %lf, target angle %lf\n", currentAngle, targetAngle);
  }

  SmartDashboard::PutNumber("Gyro", gyro->GetAngle());
}

DriveTrain::~DriveTrain() {
  // TODO Auto-generated destructor stub
}

// 0 = don't turn
// Left is positive
// Right is negative
bool DriveTrain::TurnByAngle(double degrees) {

  if (!isTurning) {
    gyro->Reset();
  }

  currentAngle = gyro->GetAngle();

  double diff = currentAngle - degrees;
  printf("Current: %lf, target: %lf Diff: %lf\n", currentAngle, degrees, diff);

  if (fabs(diff) < ROTATION_TOLERANCE_DEGREES) {
    isTurning = false;
    leftSpeed = 0;
    rightSpeed = 0;
  } else if (diff < 0) {
    // Turn right
    leftSpeed = ROTATION_SPEED;
    rightSpeed = -ROTATION_SPEED;
    isTurning = true;
  } else {
    // Turn left
    leftSpeed = -ROTATION_SPEED;
    rightSpeed = ROTATION_SPEED;
    isTurning = true;
  }
  
  return !isTurning;
}

// Forward or backward is determined by inches
// Speed should always be positive
bool DriveTrain::DriveStraight(double inches, double speed) {
  if (!isDrivingStraight) {
    leftDriveEncoder->Reset();
    rightDriveEncoder->Reset();
    startingLeftEncoder = leftDriveEncoder->Get();
    isDrivingStraight = true;
    targetAngle = fmod(gyro->GetAngle(), 360) - 180;
  }

  double correction, currentAngle;
  currentAngle = fmod(gyro->GetAngle(), 360) - 180;

  double error = currentAngle - targetAngle;
  correction = (error / 180) * 3.0;

  if (inches > 0) {
    if ((leftDriveEncoder->Get() - startingLeftEncoder) > DRIVE_DISTANCE_INCHES(inches)) {
      isDrivingStraight = false;
    }
    speed = DRIVE_FORWARD_SPEED(speed);
  } else {
    if ((leftDriveEncoder->Get() - startingLeftEncoder) < DRIVE_DISTANCE_INCHES(inches)) {
      isDrivingStraight = false;
    }

    speed = DRIVE_BACKWARD_SPEED(speed);
  }

  if (isDrivingStraight) {
    leftSpeed = speed - correction;
    rightSpeed = speed + correction;
  } else {
    leftSpeed = 0;
    rightSpeed = 0;
  }

  return !isDrivingStraight;
}

void DriveTrain::LockShifterInGear(ShifterGear gear) {
  isShifterLocked = true;
  currentGear = gear;
}

void DriveTrain::UnlockShifterGear() {
  isShifterLocked = false;
}

void DriveTrain::SetDriveMode(DriveMode newDriveMode) {
  currentDriveMode = newDriveMode;
}
