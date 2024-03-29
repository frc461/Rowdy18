/*
 * AutoCenterGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoCenterGear.h"

AutoCenterGear::AutoCenterGear(DriveTrain* driveTrain) {
  this->driveTrain = driveTrain;
  Initialize();
}

void AutoCenterGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
}

void AutoCenterGear::Execute() {
  switch(state) {
  case forward:
    if (driveTrain->DriveStraight(-75)) ++state;
    break;

  case finished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    break;
  }

  driveTrain->Execute();
}

void AutoCenterGear::Log() {

}

AutoCenterGear::~AutoCenterGear() {
  
}
