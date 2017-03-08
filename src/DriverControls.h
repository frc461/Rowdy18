/*
 * DriverControls.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_DRIVERCONTROLS_H_
#define SRC_DRIVERCONTROLS_H_

#include <WPILib.h>

enum ShifterGear {
  kLowGear,
  kHighGear
};

class DriverControls {
 public:
  DriverControls(int channel);
  virtual ~DriverControls();
  double GetLeft();
  double GetRight();
  ShifterGear GetGear();

 private:
  Joystick *joystick;
};

#endif /* SRC_DRIVERCONTROLS_H_ */
