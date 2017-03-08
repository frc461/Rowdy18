/*
 * OperatorControls.h
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#ifndef SRC_OPERATORCONTROLS_H_
#define SRC_OPERATORCONTROLS_H_

#include <WPILib.h>

enum Direction {
  kForward,
  kBackward,
  kOff
};

class OperatorControls {
public:
  OperatorControls(int port);
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
  
private:
  Joystick *joystick;
};

#endif /* SRC_OPERATORCONTROLS_H_ */
