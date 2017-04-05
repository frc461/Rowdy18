/*
 * GearManipulator.cpp
 *
 *  Created on: Apr 3, 2017
 *      Author: WBI
 */

#include "GearManipulator.h"
#include "Ports.h"

GearManipulator::GearManipulator(OperatorControls *controls) {
  this->controls = controls;

  gearEject = new DoubleSolenoid(gearEjectForwardPCM, gearEjectReversePCM);

  Initialize();
}

void GearManipulator::Initialize() {
  PeriodicExecutable::Initialize();
  direction = DoubleSolenoid::Value::kOff;

}

void GearManipulator::Execute() {
  switch(controls->GetGearDirection()) {
  case Direction::kBackward:
    direction = DoubleSolenoid::Value::kReverse;
    break;

  case Direction::kForward:
    direction = DoubleSolenoid::Value::kForward;
    break;

  case Direction::kOff:
    direction = DoubleSolenoid::Value::kOff;
    break;

  }
  gearEject->Set(direction);
}

void GearManipulator::Log() {
  Logger::Log(logGearManipulator, "Gear Eject direction: %d\n", direction);
}


GearManipulator::~GearManipulator() {
}

