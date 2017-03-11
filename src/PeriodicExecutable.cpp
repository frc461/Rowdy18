/*
 * PeriodicExecutable.cpp
 *
 *  Created on: Mar 8, 2017
 *      Author: WBI
 */

#include "PeriodicExecutable.h"

PeriodicExecutable::PeriodicExecutable() {
  timer = new Timer();
}

void PeriodicExecutable::Initialize() {
  timer->Reset();
  timer->Stop();
  isWaiting = false;
}

bool PeriodicExecutable::Wait(double seconds) {
  if (!isWaiting) {
    timer->Reset();
    timer->Start();
  }

  if (timer->Get() > seconds) {
    isWaiting = false;
  } else {
    isWaiting = true;
  }

  return !isWaiting;
}
