/*
 * AutoBase.h
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#ifndef SRC_AUTOBASE_H_
#define SRC_AUTOBASE_H_

#include <WPILib.h>
#include "PeriodicExecutable.h"

class AutoBase: public PeriodicExecutable {
 public:
  AutoBase();
  virtual ~AutoBase();
 protected:
  int state;
  
};

#endif /* SRC_AUTOBASE_H_ */
