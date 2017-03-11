/*
 * Shooter.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#include "Shooter.h"

#include "BetterPIDController.h"
#include "SettablePIDOut.h"
#include "RateCounter.h"

#define MAX_RPM 6000
#define LEFT_TOLERANCE 50
#define RIGHT_TOLERANCE 50

#define TOWER_SPEED -0.70
#define CONVEYOR_SPEED 0.90

Shooter::Shooter(OperatorControls *controls) {
  leftTower = new Spark(leftTowerPWM);
  rightTower = new Spark(rightTowerPWM);
  
  leftShooter = new Spark(leftShooterPWM);
  rightShooter = new Spark(rightShooterPWM);
  
  leftShooterEncoder = new RateCounter(leftShooterEncoderA);
  rightShooterEncoder = new RateCounter(rightShooterEncoderA);
  
  leftOut = new SettablePIDOut();
  rightOut = new SettablePIDOut();

  leftPID = new BetterPIDController(0, 0.00001, 0, leftShooterEncoder, leftOut, 0.05);
  rightPID = new BetterPIDController(0, 0.00001, 0, rightShooterEncoder, rightOut, 0.05);
  leftPID->SetSetpoint(shootingSpeed * MAX_RPM);
  rightPID->SetSetpoint(shootingSpeed * MAX_RPM);

  conveyor = new Spark(conveyorPWM);

  this->controls = controls;

  Initialize();
}

void Shooter::Initialize() {
  PeriodicExecutable::Initialize();
  usePIDForManualShooting = true;
  conveyorSpeed = 0;
}

void Shooter::Execute() {
  shootingSpeed = (controls->ScaledShootingDial() * MAX_RPM);
  leftPID->SetSetpoint(shootingSpeed);
  rightPID->SetSetpoint(shootingSpeed);

  if (controls->IsShooting() && controls->IsAutomaticShootingMode()) {
    // Towers and conveyor under automatic control
    AutomaticShooting();
  } else if (controls->IsAllOnShooting()) {
    // Towers and conveyor running
    AllOnManualShooting();
  } else {
    // Towers and conveyor under manual control

    if (controls->IsShooting()) {
      Shoot();
    } else {
      StopShooting();
    }

    RunConveyorManually();
    RunTowersManually();
  }  
}

void Shooter::AutomaticShooting() {
  Shoot();

  conveyor->SetSpeed(CONVEYOR_SPEED);
  if (fabs(leftShooterEncoder->GetRPM() - (shootingSpeed)) < LEFT_TOLERANCE) { //change to fit new encoders
    leftTower->SetSpeed(TOWER_SPEED);
  } else {
    leftTower->SetSpeed(0);
  }

  if (fabs(rightShooterEncoder->GetRPM() - (shootingSpeed)) < RIGHT_TOLERANCE) { //change to fit new encoders
    rightTower->SetSpeed(-TOWER_SPEED);
  } else {
       rightTower->SetSpeed(0);
    }
}

void Shooter::AllOnManualShooting() {
  
  leftTower->SetSpeed(TOWER_SPEED);
  rightTower->SetSpeed(-TOWER_SPEED);
  conveyor->SetSpeed(CONVEYOR_SPEED);
  
  Shoot();

}

void Shooter::Shoot() {
  if (usePIDForManualShooting) {
    if (!leftPID->IsEnabled()) {
      leftPID->Enable();
    }
    if (!rightPID->IsEnabled()) {
      rightPID->Enable();
    }

    leftShooter->SetSpeed(-leftOut->m_output);
    rightShooter->SetSpeed(rightOut->m_output);
  } else {
    leftShooter->SetSpeed(-(shootingSpeed / MAX_RPM) + .08);
    rightShooter->SetSpeed(shootingSpeed / MAX_RPM);
  }
}

void Shooter::StopShooting() {
  leftPID->Disable();
  rightPID->Disable();
  leftPID->Reset();
  rightPID->Reset();
  leftShooter->SetSpeed(0);
  rightShooter->SetSpeed(0);
}

void Shooter::Log() {
  SmartDashboard::PutNumber("Left shooter encoder", leftShooterEncoder->GetRPM());
  SmartDashboard::PutNumber("Right shooter encoder", rightShooterEncoder->GetRPM());

  const char *modeString;
  switch (mode) {
  case Automatic:
    modeString = "Automatic";
    break;
    
  case AllOnManual:
    modeString = "All on manual";
    break;

  case Manual:
    modeString = "Manual";
    break;
  }
  
  Logger::Log(logShooter, "Mode: %s\n", modeString);
  Logger::Log(logShooter, "Tower speed: %lf\n", towerSpeed);
  Logger::Log(logShooter, "Conveyor speed: %lf\n", conveyorSpeed);
  Logger::Log(logShooter, "Shooting speed: %lf\n", shootingSpeed);
  Logger::LogPID(logShooter, "LeftPID", leftPID);
  Logger::LogPID(logShooter, "RightPID", rightPID);
}

void Shooter::RunTowersManually() {
  switch (controls->GetTowerDirection()) {
  case Direction::kForward:
    towerSpeed = TOWER_SPEED;
    break;

  case Direction::kBackward:
    towerSpeed = -TOWER_SPEED;
    break;

  case Direction::kOff:
    towerSpeed = 0;
    break;
  }
  leftTower->SetSpeed(towerSpeed);
  rightTower->SetSpeed(-towerSpeed);
}

void Shooter::RunConveyorManually() {
    switch (controls->GetConveyorDirection()) {
    case Direction::kForward:
      conveyorSpeed = CONVEYOR_SPEED;
      break;

    case Direction::kBackward:
      conveyorSpeed = -CONVEYOR_SPEED;
      break;

    case Direction::kOff:
      conveyorSpeed = 0;
      break;
    }
    conveyor->SetSpeed(conveyorSpeed);
}
