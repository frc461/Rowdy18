/*
 * Catapult.h
 *
 *  Created on: Apr 19, 2017
 *      Author: WBI
 */

#ifndef SRC_CATAPULT_H_
#define SRC_CATAPULT_H_

#include "PeriodicExecutable.h"

class Catapult: public PeriodicExecutable {
public:
  Catapult(OperatorControls *controls);

  void Initialize();
  void Execute();
  void Log();
private:
  DoubleSolenoid *solenoid1;
  DoubleSolenoid *solenoid2;
  OperatorControls *controls;
};

#endif /* SRC_CATAPULT_H_ */
