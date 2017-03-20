/*
 * DriverControls.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "DriverControls.h"
#include "XboxJoystickMap.h"
#include "Ports.h"

#define DRIVE_DEADZONE 0.1

DriverControls::DriverControls(int channel) {
  joystick = new Joystick(channel);
}

DriverControls *DriverControls::driverControls = NULL;

DriverControls *DriverControls::SharedDriverControls() {
  if (!driverControls) {
    driverControls = new DriverControls(Joysticks::DriverControlsJoystick);
  }

  return driverControls;
}

double DriverControls::GetLeft() {
  double val = joystick->GetRawAxis(XboxAxisLeftStickY);
  return fabs(val) > DRIVE_DEADZONE ? val : 0;
}

double DriverControls::GetRight() {
  double val = joystick->GetRawAxis(XboxAxisRightStickY);
  return fabs(val) > DRIVE_DEADZONE ? val : 0;
}

ShifterGear DriverControls::GetGear() {
  if (joystick->GetRawAxis(shiftGearsAxisXbox) > 0.5)
    return ShifterGear::kHighGear;
  else
    return ShifterGear::kLowGear;
}

bool DriverControls::GetClimber() {
  bool shouldClimb = joystick->GetRawButton(climberButtonXbox);
  printf("Should climb: %s\n", shouldClimb ? "true" : "false");
  return shouldClimb;
}

DriverControls::~DriverControls() {
  // TODO Auto-generated destructor stub
}

