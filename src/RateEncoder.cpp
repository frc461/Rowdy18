/*
 * RateEncoder.cpp
 *
 *  Created on: Jan 16, 2017
 *      Author: WBI
 */

#include <RateEncoder.h>

RateEncoder::RateEncoder(int aChannel, int bChannel) : Encoder(aChannel, bChannel, false, k4X) {
	// TODO Auto-generated constructor stub
}

RateEncoder::~RateEncoder() {
	// TODO Auto-generated destructor stub
}

double RateEncoder::PIDGet() {
	return this->GetRate();
}

