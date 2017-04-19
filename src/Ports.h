/*
 * Ports.h
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#ifndef SRC_PORTS_H_
#define SRC_PORTS_H_

#include "XboxJoystickMap.h"

enum Joysticks {
  DriverControlsJoystick = 0,
  OperatorControlsJoystick = 1
};

enum PWM {
	frontLeftPWM = 0,
	frontRightPWM = 2,
	backLeftPWM = 1,
	backRightPWM = 3,
	intakeRollerPWM = 4,
	climberPWM = 6,
	leftShooterPWM = 9, //shooter 1
	leftTowerPWM = 10,
	rightShooterPWM = 8, //shooter 2
	rightTowerPWM = 7,
	conveyorPWM = 5
};

enum PCM {
	intakeForwardPCM = 2,
	intakeReversePCM = 3,
	shifterForwardPCM = 0,
	shifterReversePCM = 1,
	gearEjectForwardPCM = 4,
	gearEjectReversePCM = 5
};

enum OperatorControlInputs {
	spinIntakeForwardButton = 11,
	spinIntakeBackwardButton = 12,
	climberButton = 2,
	towersInButton = 7,
	towersOutButton = 8,
	shootingTowersConveyorButton = 1,
	shootingButton = 3,
	changeShooterSpeed = 0,
	shootingModeSwitch = 6,
	intakePositionSwitch = 5,
	conveyorIn = 10,
	conveyorOut = 9,
	gearEjectButton = 4,
	catapultTrigger = XboxButtonX

};
/*
enum HatControls {
	lowerIntakePOV = 270,
	raiseIntakePOV = 90,
	conveyorInPOV = 0,
	conveyorOutPOV = 180
};
*/

enum DriverControlsXbox {
	shiftGearsAxisXbox = 3,
	climberButtonXbox = XboxButtonLeftBumper,
	cameraSelectionXbox = XboxButtonRightBumper
};

enum DriverControlsRightJoystick {
	shiftGearsButtonRightJoystick = 1,
};

enum DriverControlsLeftJoystick {
	driveStraightButtonLeftJoystick = 1,
};

enum DIO {	//digital input/output
	leftDriveEncoderA = 1,
	leftDriveEncoderB = 0,
	rightDriveEncoderA = 2,
	rightDriveEncoderB = 3,
	leftShooterEncoderA = 4,
	leftShooterEncoderB = 5,
	rightShooterEncoderA = 6,
	rightShooterEncoderB = 7,
	leftLimitSwitchDIO = 8,
	rightLimitSwitchDIO = 9
};

enum PDP_CHANNELS {
	pdpConveyor = 7,
	pdpIntake = 5
};

#endif /* SRC_PORTS_H_ */
