/*
 * Climber.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "Climber.h"
#include "Ports.h"

#define CLIMBER_SPEED -1.0

Climber::Climber(OperatorControls *controls) {
  this->controls = controls;

  climber = new Spark(climberPWM);
}

void Climber::Execute() {
  switch(controls->GetClimberDirection()) {
  case Direction::kBackward:
    climber->SetSpeed(-CLIMBER_SPEED);
    break;

  case Direction::kForward:
    climber->SetSpeed(CLIMBER_SPEED);
    break;

  case Direction::kOff:
    climber->SetSpeed(0);
    break;
  }
}

void Climber::Log() {

}
Climber::~Climber() {
  // TODO Auto-generated destructor stub
}

