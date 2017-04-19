/*
 * AutoRightGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoRightGear.h"

AutoRightGear::AutoRightGear(DriveTrain* driveTrain, GearManipulator *gearManipulator) {
  this->driveTrain = driveTrain;
  this->gearManipulator = gearManipulator;
  Initialize();
}

void AutoRightGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  startingAngle = 0;
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
  driveTrain->SetDriveMode(DriveTrain::DriveMode::tank);
}

void AutoRightGear::Execute() {
  printf("State: %d\n", state);

  switch(state) {
  case rgForward0:
    if (driveTrain->DriveStraight(-92, .6)) ++state;
    startingAngle = driveTrain->gyro->GetAngle();
    break;

  case rgTurning:
    if (driveTrain->TurnByAngle(58)) ++state;
    break;

  case rgForward1:
    if (driveTrain->DriveStraight(-18, .6)) ++state;
    break;
    
  case rgPunch:
//    gearManipulator->Pow();
    ++state;
    break;

  case rgFinished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    break;
  }

  driveTrain->Execute();
}

void AutoRightGear::Log() {
  Logger::Log(logAuton, "State: %d\n", state);
  driveTrain->Log();
}

AutoRightGear::~AutoRightGear() {
  
}
