/*
 * Intake.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "Intake.h"
#include "Ports.h"

#define ROLLER_SPEED 0.9

#define INTAKE_DOWN DoubleSolenoid::kReverse
#define INTAKE_UP DoubleSolenoid::kForward

Intake::Intake(OperatorControls *controls) {
  // TODO Auto-generated constructor stub

  this->controls = controls;

  intakeRoller = new Spark(intakeRollerPWM);
  intake = new DoubleSolenoid(intakeForwardPCM, intakeReversePCM);
}

void Intake::Initialize() {
  PeriodicExecutable::Initialize();
}

void Intake::Execute() {
  switch (controls->GetIntakeDirection()) {
  case Direction::kBackward:
    intake->Set(INTAKE_DOWN);
    break;

  case Direction::kForward:
  default:
    intake->Set(INTAKE_UP);
  }

  switch (controls->GetIntakeRollerDirection()) {
  case Direction::kForward:
    intakeRoller->SetSpeed(ROLLER_SPEED);
    break;

  case Direction::kBackward:
    intakeRoller->Set(-ROLLER_SPEED);
    break;

  case Direction::kOff:
    intakeRoller->Set(0);
    break;
  }
}

void Intake::Log() {

}

Intake::~Intake() {
  // TODO Auto-generated destructor stub
}

