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

enum CameraSelect {
  kCamera0,
  kCamera1
};

class DriverControls {
 public:
  virtual ~DriverControls();
  double GetLeft();
  double GetRight();
  ShifterGear GetGear();
  bool GetClimber();
  CameraSelect GetCameraSelect();

  static DriverControls *SharedDriverControls();

 private:
  DriverControls(int channel);
  Joystick *joystick;
  static DriverControls *driverControls;
};

#endif /* SRC_DRIVERCONTROLS_H_ */
