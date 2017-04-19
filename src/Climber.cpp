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

  Initialize();
}

void Climber::Initialize() {
  PeriodicExecutable::Initialize();
  speed = 0;
}

void Climber::Execute() {
  switch(controls->GetClimberDirection()) {
  case Direction::kBackward:
//    speed = -CLIMBER_SPEED;
//    break;

  case Direction::kForward:
    speed = CLIMBER_SPEED;
    break;

  case Direction::kOff:
    speed = 0;
    break;
  }

  climber->SetSpeed(speed);
}

void Climber::Log() {

  // Climbing up is negative
  Logger::Log(logClimber, "Climber speed: %lf\n", -speed);
}
Climber::~Climber() {
  // TODO Auto-generated destructor stub
}

