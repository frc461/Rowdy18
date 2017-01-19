#include "XboxJoystickMap.h"

enum RightGearHighGoal {
	rGHG_LowGear,
	rGHG_BackUp0,
	rGHG_RotateLeft,
	rGHG_BackUp1,
	rGHG_PlaceGear,
	rGHG_DriveForward,
	rGHG_ShootFuel
};

enum RightGearHighGoalReload {
	rGHGR_RGHG = rGHG_ShootFuel,
	rGHGR_BackUp0,
	rGHGR_LowerIntake,
	rGHGR_DriveToHopper
};

enum LeftGearReload {
	lGR_LowGear,
	lGR_BackUp0,
	lGR_RotateRight0,
	lGR_BackUp1,
	lGR_PlaceGear,
	lGR_DriveForward,
	lGR_RotateRight1,
	lGR_LowerIntake,
	lGR_DriveToHopper
};

enum AutoMode {
	rightGearHighGoal,
	rightGearHighGoalReload,
	centerGear,
	leftGearReload,
	highGoalReload
};

enum PWM {
	frontLeftPWM,
	frontRightPWM,
	backLeftPWM,
	backRightPWM,
	intakeForwardPWM,
	intakeReversePWM,
	intakeRollerPWM,
	climberUpPWM,
	shifterForwardPWM,
	shifterReversePWM,
	leftShooterPWM,
	leftFeederPWM,
	rightShooterPWM,
	rightFeederPWM
};

enum OperatorControls {
	lowerIntakeButton = XboxButtonA,
	raiseIntakeButton = XboxButtonB,
	spinIntakeForwardButton = XboxButtonX,
	spinIntakeBackwardButton = XboxButtonY,
	climberUpButton = XboxButtonLeftBumper,
	shootingButton = XboxAxisLeftTrigger
};

enum DriverControls {
	shiftGearsButton = 0
};

enum DIO {	//digital input/output
	leftEncoderA,
	leftEncoderB,
	rightEncoderA,
	rightEncoderB
};
