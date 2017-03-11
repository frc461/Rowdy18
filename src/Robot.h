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
	lGR_Init,
	lGR_LowGear,
	lGR_BackUp0Init,
	lGR_BackUp0,
	lGR_LowerIntake,
	lGR_BackUp1Init,
	lGR_BackUp1,
	lGR_RotateRight0Init,
	lGR_RotateRight0,
	lGR_BackUp2Init,
	lGR_BackUp2
};

enum autoLGRHopperRed{
	lGRHR_LeftGearReload = lGR_BackUp2,
	lGRHR_Init,
	lGRHR_DriveForward0,
	lGRHR_RotateRight0Init,
	lGRHR_RotateRight0,
	lGRHR_DriveForward1
};

enum autoLGRHopperBlue{
	lGRHB_LeftGearReload = lGR_BackUp2,
	lGRHB_Init,
	lGRHB_DriveForward0,
	lGRHB_RotateRight0Init,
	lGRHB_RotateRight0,
	lGRHB_DriveForward1,
	lGRHB_RotateLeft0Init,
	lGRHB_RotateLeft0,
	lGRHB_DriveForward2
};

enum RightGearReload {
	rGR_Init,
	rGR_LowGear,
	rGR_BackUp0Init,
	rGR_BackUp0,
	rGR_LowerIntake,
	rGR_BackUp1Init,
	rGR_BackUp1,
	rGR_RotateRight0Init,
	rGR_RotateRight0,
	rGR_BackUp2Init,
	rGR_BackUp2
};

enum autoRGRHopperRed{
	rGRHR_RightGearReload = rGR_BackUp2,
	rGRHR_Init,
	rGRHR_DriveForward0,
	rGRHR_RotateLeft0Init,
	rGRHR_RotateLeft0,
	rGRHR_DriveForward1,
	rGRHB_RotateRight0Init,
	rGRHB_RotateRight0,
	rGRHR_DriveForward2
};

enum autoRGRHopperBlue{
	rGRHB_RightGearReload = rGR_BackUp2,
	rGRHB_Init,
	rGRHB_DriveForward0,
	rGRHB_RotateLeft0Init,
	rGRHB_RotateLeft0,
	rGRHB_DriveForward1
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

enum CenterGear {
	CG_Init,
	CG_Drive,
	CG_FINISH
};

enum AutoMode {
	rightGearHighGoal,
	rightGearHighGoalReload,
	centerGear,
	leftGearReload,
	lGRHopperRed,
	lGRHopperBlue,
	rightGearReload,
	rGRHopperRed,
	rGRHopperBlue,
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
	climberDownSwitch = 4 // TODO: Implement hood
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
	rightShooterEncoderB = 7
};

enum PDP_CHANNELS {
	pdpConveyor = 7,
	pdpIntake = 5
};
