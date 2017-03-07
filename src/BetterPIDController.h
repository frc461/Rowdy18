/*
 * MyPIDController.h
 *
 *  Created on: Mar 3, 2017
 *      Author: WBI
 */

#ifndef BETTERPIDCONTROLLER_H_
#define BETTERPIDCONTROLLER_H_

#include "WPILib.h"

class BetterPIDController: public PIDController {
public:
	BetterPIDController(double p, double i, double d, PIDSource* source,
	                PIDOutput* output, double period = 0.05);
	BetterPIDController(double p, double i, double d, double f, PIDSource* source,
	                PIDOutput* output, double period = 0.05);
	virtual ~BetterPIDController();

	double GetInput();
};

#endif /* BETTERPIDCONTROLLER_H_ */
