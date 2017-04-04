/*
 * GearManipulator.h
 *
 *  Created on: Apr 3, 2017
 *      Author: WBI
 */

#ifndef GEARMANIPULATOR_H_
#define GEARMANIPULATOR_H_

#include <WPILib.h>
#include "PeriodicExecutable.h"

class GearManipulator: public PeriodicExecutable {
public:
  GearManipulator(OperatorControls *controls);

  void Initialize();
  void Execute();
  void Log();
  virtual ~GearManipulator();

private:
  OperatorControls *controls;
  DoubleSolenoid *gearEject;
  DoubleSolenoid::Value direction;
};

#endif /* GEARMANIPULATOR_H_ */
