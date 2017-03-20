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

enum State {
 forward,
 finished
};

class AutoCenterGear: public AutoBase {
 public:
  AutoCenterGear(DriveTrain* driveTrain);
  void Initialize() override;
  virtual ~AutoCenterGear();

  void Execute() override;
  void Log();
 private:
  DriveTrain* driveTrain;
};

#endif /* SRC_AUTOCENTERGEAR_H_ */
