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
	leftTowerPWM = 6,
	rightShooterPWM = 9,
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
	spinIntakeForwardButton = 6,
	spinIntakeBackwardButton = 5,
	climberButton = 3,
	shootingButton = 9,
	towersInButton = 2,
	towersOutButton = 1,
	manualShootingButton = 4,
	changeShooterSpeed = 0,
	shootingModeSwitch = 7
};

enum HatControls {
	lowerIntakePOV = 270,
	raiseIntakePOV = 90,
	conveyorInPOV = 0,
	conveyorOutPOV = 180
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
