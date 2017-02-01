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

enum HighGoalReload {
	HGR_DriveForward0,
	HGR_RotateRight0,
	HGR_DriveForward1,
	HGR_RotateRight1,
	HGR_LowerIntake,
	HGR_RunIntake,
	HGR_DriveBackwards,
	HGR_RotateRight2,
	HGR_DriveForward,
	HGR_Shoot

};

enum AutoMode {
	rightGearHighGoal,
	rightGearHighGoalReload,
	centerGear,
	leftGearReload,
	highGoalReload
};

enum PWM {
	frontLeftPWM = 0,
	frontRightPWM = 2,
	backLeftPWM = 1,
	backRightPWM = 3,
	intakeRollerPWM,
	climberPWM,
	leftShooterPWM = 8,
	leftFeederPWM = 6,
	rightShooterPWM = 9,
	rightFeederPWM = 7
};

enum PCM {
	intakeForwardPCM = 2,
	intakeReversePCM = 3,
	shifterForwardPCM = 0,
	shifterReversePCM = 1
};

enum OperatorControls {
	lowerIntakeButton = XboxButtonA,
	raiseIntakeButton = XboxButtonB,
	spinIntakeForwardButton = XboxButtonX,
	spinIntakeBackwardButton = XboxButtonY,
	climberButton = XboxButtonLeftBumper,
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
