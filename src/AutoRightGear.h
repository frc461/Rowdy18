/*
 * AutoRightGear.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_AUTORIGHTGEAR_H_
#define SRC_AUTORIGHTGEAR_H_

#include <WPILib.h>
#include "AutoBase.h"
#include "DriveTrain.h"
#include "GearManipulator.h"

enum RightGearState {
 rgForward0,
 rgTurning,
 rgForward1,
 rgPunch,
 rgFinished
};

class AutoRightGear: public AutoBase {
 public:
  AutoRightGear(DriveTrain* driveTrain, GearManipulator *gearManipulator);
  void Initialize() override;
  virtual ~AutoRightGear();

  void Execute() override;
  void Log();
 private:
  DriveTrain* driveTrain;
  GearManipulator *gearManipulator;
  double startingAngle;
};

#endif /* SRC_AUTORIGHTGEAR_H_ */
