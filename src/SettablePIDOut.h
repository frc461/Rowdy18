/*
 * SettablePIDOut.h
 *
 *  Created on: Feb 14, 2017
 *      Author: WBI
 */

#ifndef SRC_SETTABLEPIDOUT_H_
#define SRC_SETTABLEPIDOUT_H_

#include "WPILib.h"

class SettablePIDOut : public PIDOutput {
public:
	SettablePIDOut();
	virtual ~SettablePIDOut();

	void PIDWrite(double output);
	double m_output;
};

#endif /* SRC_SETTABLEPIDOUT_H_ */
