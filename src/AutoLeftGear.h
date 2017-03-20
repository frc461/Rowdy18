/*
 * AutoLeftGear.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_AUTOLEFTGEAR_H_
#define SRC_AUTOLEFTGEAR_H_

#include <WPILib.h>
#include "AutoBase.h"
#include "DriveTrain.h"

enum LeftGearState {
 lgForward0,
 lgTurning,
 lgForward1,
 lgFinished
};

class AutoLeftGear: public AutoBase {
 public:
  AutoLeftGear(DriveTrain* driveTrain);
  void Initialize() override;
  virtual ~AutoLeftGear();

  void Execute() override;
  void Log();
 private:
  DriveTrain* driveTrain;
  double startingAngle;
};

#endif /* SRC_AUTOLEFTGEAR_H_ */
