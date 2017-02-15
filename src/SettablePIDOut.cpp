/*
 * SettablePIDOut.cpp
 *
 *  Created on: Feb 14, 2017
 *      Author: WBI
 */

#include <SettablePIDOut.h>

SettablePIDOut::SettablePIDOut() {
	// TODO Auto-generated constructor stub

}

SettablePIDOut::~SettablePIDOut() {
	// TODO Auto-generated destructor stub
}

void SettablePIDOut::PIDWrite(double output) {
	m_output = output;
}

