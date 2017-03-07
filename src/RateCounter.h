/*
 * RateCounter.h
 *
 *  Created on: Feb 20, 2017
 *      Author: WBI
 */

#ifndef SRC_RATECOUNTER_H_
#define SRC_RATECOUNTER_H_
#include "WPILib.h"

class RateCounter: public frc::Counter, public frc::PIDSource {
public:
	RateCounter(int channel, double adjustmentFactor = 1);
	virtual ~RateCounter();
	double PIDGet();
	double GetRPM();

private:
	double m_adjustmentFactor;
};

#endif /* SRC_RATECOUNTER_H_ */
