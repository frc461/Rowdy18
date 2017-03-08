/*
 * Intake.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_INTAKE_H_
#define SRC_INTAKE_H_

#include <WPILib.h>
#include "PeriodicExecutable.h"

class Intake: public PeriodicExecutable {
 public:
  Intake(OperatorControls *controls);
  virtual ~Intake();
  
  void Execute();
  void Log();

 private:
  OperatorControls *controls;
  Spark *intakeRoller;
  DoubleSolenoid *intake;
};

#endif /* SRC_INTAKE_H_ */
