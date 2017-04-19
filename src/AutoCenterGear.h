/*
 * AutoCenterGear.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_AUTOCENTERGEAR_H_
#define SRC_AUTOCENTERGEAR_H_

#include <WPILib.h>
#include "AutoBase.h"
#include "DriveTrain.h"
#include "GearManipulator.h"

enum State {
 forward = 0,
 punch = 1,
 backup = 2,
 finished = 3
};

class AutoCenterGear: public AutoBase {
 public:
  AutoCenterGear(DriveTrain* driveTrain, GearManipulator *gearManipulator);
  void Initialize() override;
  virtual ~AutoCenterGear();

  void Execute() override;
  void Log();
 private:
  DriveTrain* driveTrain;
  GearManipulator *gearManipulator;
};

#endif /* SRC_AUTOCENTERGEAR_H_ */
