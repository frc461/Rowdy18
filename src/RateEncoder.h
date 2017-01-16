/*
 * RateEncoder.h
 *
 *  Created on: Jan 16, 2017
 *      Author: WBI
 */

#ifndef SRC_RATEENCODER_H_
#define SRC_RATEENCODER_H_

#include <Encoder.h>

class RateEncoder: public frc::Encoder {
public:
	RateEncoder(int aChannel, int bChannel);
	virtual ~RateEncoder();

	double PIDGet();
};

#endif /* SRC_RATEENCODER_H_ */
