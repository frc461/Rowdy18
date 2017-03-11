/*
 * AutoCenterGear.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "AutoCenterGear.h"

AutoCenterGear::AutoCenterGear(DriveTrain* driveTrain) {
  this->driveTrain = driveTrain;
  
}

void AutoCenterGear::Execute() {
  switch(state) {
  case forward:
    if (driveTrain->DriveStraight(65)) ++state;
    break;

  case finished:
    driveTrain->DriveStraight(0, 0);
    break;
  }

  driveTrain->Execute();
}

void AutoCenterGear::Log() {

}

AutoCenterGear::~AutoCenterGear() {
  
}
