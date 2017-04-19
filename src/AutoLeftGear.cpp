/*
 * AutoLeftGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoLeftGear.h"

AutoLeftGear::AutoLeftGear(DriveTrain* driveTrain, GearManipulator *gearManipulator) {
  this->driveTrain = driveTrain;
  this->gearManipulator = gearManipulator;
  Initialize();
}

void AutoLeftGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  startingAngle = 0;
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
  driveTrain->SetDriveMode(DriveTrain::DriveMode::tank);
}

void AutoLeftGear::Execute() {
  printf("State: %d\n", state);

  switch(state) {
  case lgForward0:
    if (driveTrain->DriveStraight(-92, .6)) ++state;
    startingAngle = driveTrain->gyro->GetAngle();
    break;

  case lgTurning:
    if (driveTrain->TurnByAngle(-58)) ++state;
    break;

  case lgForward1:
    if (driveTrain->DriveStraight(-18, .6)) ++state;
    break;
    
  case lgPunch:
//    gearManipulator->Pow();
    ++state;
    break;

  case lgFinished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    break;
  }

  driveTrain->Execute();
}

void AutoLeftGear::Log() {
  Logger::Log(logAuton, "State: %d\n", state);
  driveTrain->Log();
}

AutoLeftGear::~AutoLeftGear() {
  
}
