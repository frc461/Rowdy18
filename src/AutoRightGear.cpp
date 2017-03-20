/*
 * AutoRightGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoRightGear.h"

AutoRightGear::AutoRightGear(DriveTrain* driveTrain) {
  this->driveTrain = driveTrain;
  Initialize();
}

void AutoRightGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  startingAngle = 0;
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
}

void AutoRightGear::Execute() {
  printf("State: %d\n", state);

  switch(state) {
  case rgForward0:
    if (driveTrain->DriveStraight(-95)) ++state;
    startingAngle = driveTrain->gyro->GetAngle();
    break;

  case rgTurning:
    if (driveTrain->TurnByAngle(65)) ++state;
    break;

  case rgForward1:
    if (driveTrain->DriveStraight(-18)) ++state;
    break;
    
  case rgFinished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    break;
  }

  driveTrain->Execute();
}

void AutoRightGear::Log() {

}

AutoRightGear::~AutoRightGear() {
  
}
