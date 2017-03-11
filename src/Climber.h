/*
 * Climber.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_CLIMBER_H_
#define SRC_CLIMBER_H_

#include <WPILib.h>
#include "PeriodicExecutable.h"

class Climber: public PeriodicExecutable {
 public:
  Climber(OperatorControls *controls);

  void Initialize();
  void Execute();
  void Log();
  virtual ~Climber();

 private:
  OperatorControls *controls;
  Spark *climber;
  double speed;
};

#endif /* SRC_CLIMBER_H_ */
