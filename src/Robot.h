#include "XboxJoystickMap.h"

#include "Ports.h"

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
