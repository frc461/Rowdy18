/*
 * AutoLeftGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoLeftGear.h"

AutoLeftGear::AutoLeftGear(DriveTrain* driveTrain) {
  this->driveTrain = driveTrain;
  Initialize();
}

void AutoLeftGear::Initialize() {
  AutoBase::Initialize();
  state = 0;
  driveTrain->Initialize();
  startingAngle = 0;
  driveTrain->LockShifterInGear(ShifterGear::kLowGear);
}

void AutoLeftGear::Execute() {
  printf("State: %d\n", state);

  switch(state) {
  case lgForward0:
    if (driveTrain->DriveStraight(-95)) ++state;
    startingAngle = driveTrain->gyro->GetAngle();
    break;

  case lgTurning:
    if (driveTrain->TurnByAngle(-65)) ++state;
    break;

  case lgForward1:
    if (driveTrain->DriveStraight(-18)) ++state;
    break;
    
  case lgFinished:
    driveTrain->DriveStraight(0, 0);
    driveTrain->UnlockShifterGear();
    break;
  }

  driveTrain->Execute();
}

void AutoLeftGear::Log() {

}

AutoLeftGear::~AutoLeftGear() {
  
}
