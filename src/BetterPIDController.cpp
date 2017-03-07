/*
 * MyPIDController.cpp
 *
 *  Created on: Mar 3, 2017
 *      Author: WBI
 */

#include "BetterPIDController.h"

PIDSource *m_source;
BetterPIDController::BetterPIDController(double p, double i, double d, PIDSource* source, PIDOutput* output, double period) :
		PIDController (p, i, d, source, output, period) {
	// TODO Auto-generated constructor stub
	m_source = source;
}

BetterPIDController::BetterPIDController(double p, double i, double d, double f, PIDSource* source, PIDOutput* output, double period) :
	PIDController(p, i, d, f, source, output, period) {
	m_source = source;
}

double BetterPIDController::GetInput() {
	return m_source->PIDGet();
}

BetterPIDController::~BetterPIDController() {
	// TODO Auto-generated destructor stub
}

