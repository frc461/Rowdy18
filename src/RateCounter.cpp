/*
 * RateCounter.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: WBI
 */

#include <RateCounter.h>

RateCounter::RateCounter(int channel, double adjustmentFactor) : Counter(channel) {
	// TODO Auto-generated constructor stub
	m_adjustmentFactor = adjustmentFactor;
}

RateCounter::~RateCounter() {
	// TODO Auto-generated destructor stub
}

double RateCounter::PIDGet() {
	return GetRPM();
}

double RateCounter::GetRPM() {
	return 60/this->GetPeriod() * m_adjustmentFactor;
}
