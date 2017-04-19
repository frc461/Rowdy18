/*
 * OperatorControls.h
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#ifndef SRC_OPERATORCONTROLS_H_
#define SRC_OPERATORCONTROLS_H_

#include <WPILib.h>

#include "DriverControls.h"

enum Direction {
  kForward,
  kBackward,
  kOff
};

class OperatorControls {
public:
  virtual ~OperatorControls();

  // Shooting
  bool IsAutomaticShootingMode();

  bool IsAllOnShooting();
  bool IsShooting();
  double ScaledShootingDial();

  Direction GetConveyorDirection();
  Direction GetTowerDirection();

  Direction GetIntakeDirection();
  Direction GetIntakeRollerDirection();

  Direction GetClimberDirection();
  Direction GetGearDirection();
  
  Direction GetCatapultDirection();

  static OperatorControls *SharedOperatorControls();

private:
  OperatorControls(int port);
  static OperatorControls *operatorControls;
  Joystick *joystick;
  SendableChooser<bool> *climberDirectionChooser = new SendableChooser<bool>();
};

#endif /* SRC_OPERATORCONTROLS_H_ */
