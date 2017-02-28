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
	intakeRollerPWM = 4,
	climberPWM = 10,
	leftShooterPWM = 9, //shooter 1
	leftTowerPWM = 6,
	rightShooterPWM = 8, //shooter 2
	rightTowerPWM = 7,
	conveyorPWM = 5
};

enum PCM {
	intakeForwardPCM = 2,
	intakeReversePCM = 3,
	shifterForwardPCM = 0,
	shifterReversePCM = 1
};

enum OperatorControls {
	spinIntakeForwardButton = 12,
	spinIntakeBackwardButton = 11,
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
	hoodPositionSwitch = 4 // TODO: Implement hood
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
	shiftGearsAxisXbox = 3
};

enum DriverControlsRightJoystick {
	shiftGearsButtonRightJoystick = 1,
};

enum DIO {	//digital input/output
	leftDriveEncoderA = 1,
	leftDriveEncoderB = 0,
	rightDriveEncoderA = 2,
	rightDriveEncoderB = 3,
	leftShooterEncoderA = 4,
	leftShooterEncoderB = 5,
	rightShooterEncoderA = 6,
	rightShooterEncoderB = 7
};

enum PDP_CHANNELS {
	pdpConveyor = 7,
	pdpIntake = 5
};
