/*
 * LogTypes.h
 *
 *  Created on: Feb 18, 2017
 *      Author: WBI
 */

#ifndef SRC_LOGTYPES_H_
#define SRC_LOGTYPES_H_


// Use this enum to list each category you want to log

enum LOG_TYPE {
	logShooter,
	logDriveTrain,
	logAuton,
	logOperator,
	logDriver,
	logShifter,
	logConveyor,
	logTower,
	logIntake,
	logClimber,
};

// This array is used to print a prefix on each log line based on the LOG_TYPE
// IT IS VERY IMPORTANT THAT THESE ARE IN THE SAME ORDER AS THE ENUM

static const char* LOG_PREFIXES[] = {"Shooter", "DriveTrain", "Auton", "Operator", "Driver", "Shifter", "Conveyor", "Tower", "Intake", "Climber"};



#endif /* SRC_LOGTYPES_H_ */
