/*
 * RateCounter.cpp
 *
 *  Created on: Feb 20, 2017
 *      Author: WBI
 */

#include <RateCounter.h>

RateCounter::RateCounter(int channel) : Counter(channel) {
	// TODO Auto-generated constructor stub

}

RateCounter::~RateCounter() {
	// TODO Auto-generated destructor stub
}

double RateCounter::PIDGet() {
	return 60/this->GetPeriod();
}
