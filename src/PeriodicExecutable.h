/*
 * PeriodicExecutable.h
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#ifndef SRC_PERIODICEXECUTABLE_H_
#define SRC_PERIODICEXECUTABLE_H_

#include <WPILib.h>
#include "OperatorControls.h"
#include "Ports.h"

class PeriodicExecutable {
 public:
  PeriodicExecutable();
  virtual void Execute() = 0;
  virtual void Log() = 0;

  bool Wait(double seconds);

 private:
  Timer *timer;
  bool isWaiting;
};

#endif /* SRC_PERIODICEXECUTABLE_H_ */
