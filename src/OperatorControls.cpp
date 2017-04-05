/*
 * OperatorControls.cpp
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#include <OperatorControls.h>
#include "Ports.h"

OperatorControls::OperatorControls(int port) {
  // TODO Auto-generated constructor stub
  joystick = new Joystick(port);
  climberDirectionChooser->AddDefault("Climb up", kForward);
  climberDirectionChooser->AddObject("Climb down", kBackward);
  SmartDashboard::PutData("climberDirectionChooser", climberDirectionChooser);
}

OperatorControls *OperatorControls::operatorControls = NULL;

OperatorControls *OperatorControls::SharedOperatorControls() {
  if (!operatorControls) {
    operatorControls = new OperatorControls(Joysticks::OperatorControlsJoystick);
  }

  return operatorControls;
}

OperatorControls::~OperatorControls() {
  // TODO Auto-generated destructor stub
}

bool OperatorControls::IsAutomaticShootingMode() {
  return !joystick->GetRawButton(shootingModeSwitch);
}

// Scales input from [1.0, -1.0] to [0.0, 1.0]
double OperatorControls::ScaledShootingDial() {
  return (-joystick->GetRawAxis(OperatorControlInputs::changeShooterSpeed)/2.0) + 0.5;
}

bool OperatorControls::IsAllOnShooting() {
  return joystick->GetRawButton(shootingTowersConveyorButton);
}

bool OperatorControls::IsShooting() {
  return joystick->GetRawButton(shootingButton);
}

Direction OperatorControls::GetConveyorDirection() {
  if (joystick->GetRawButton(OperatorControlInputs::conveyorIn))
    return Direction::kForward;
  else if (joystick->GetRawButton(OperatorControlInputs::conveyorOut))
    return Direction::kBackward;

  return Direction::kOff;
}

Direction OperatorControls::GetTowerDirection() {
  if (joystick->GetRawButton(OperatorControlInputs::towersInButton))
    return Direction::kForward;

  else if (joystick->GetRawButton(OperatorControlInputs::towersOutButton))
    return Direction::kBackward;

  return Direction::kOff;
}

Direction OperatorControls::GetIntakeDirection() {
  if (joystick->GetRawButton(intakePositionSwitch))
    return Direction::kBackward;
  else
    return Direction::kForward;
}

Direction OperatorControls::GetIntakeRollerDirection() {
  if (joystick->GetRawButton(spinIntakeForwardButton))
    return Direction::kForward;
  else if (joystick->GetRawButton(spinIntakeBackwardButton))
    return Direction::kBackward;
  else
    return Direction::kOff;
}

Direction OperatorControls::GetClimberDirection() {
  if (joystick->GetRawButton(climberButton) || DriverControls::SharedDriverControls()->GetClimber()) {
    printf("Climbing\n");
    return climberDirectionChooser->GetSelected() ? kForward : kBackward;
  }
  return Direction::kOff;
}

Direction OperatorControls::GetGearDirection() {
  if (joystick->GetRawButton(gearEjectButton)) {
    return Direction::kBackward;
  } else {
    return Direction::kForward;
  }
}
