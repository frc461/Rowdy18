/*
 * AutoCenterGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoCenterGear.h"

AutoCenterGear::AutoCenterGear(DriveTrain* driveTrain, GearManipulator *gearManipulator) {
  this->driveTrain = driveTrain;
  this->gearManipulator = gearManipulator;
  Initialize();
}

void AutoCenterGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
  driveTrain->SetDriveMode(DriveTrain::DriveMode::tank);

  gearManipulator->Initialize();
}

void AutoCenterGear::Execute() {
  switch(state) {
  case forward:
    if (driveTrain->DriveStraight(-75, .6)) ++state;
    break;

  case punch:
    gearManipulator->Pow();
    ++state;
    break;

  case backup:
    if (driveTrain->DriveStraight(5)) ++state;
    break;

  case finished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    gearManipulator->Unpow();
    break;
  }

  driveTrain->Execute();
}

void AutoCenterGear::Log() {
  Logger::Log(logAuton, "State: %d\n", state);
  driveTrain->Log();
}

AutoCenterGear::~AutoCenterGear() {
  
}
