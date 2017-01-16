#include "XboxJoystickMap.h"

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

enum DIO {
	leftEncoderA,
	leftEncoderB,
	rightEncoderA,
	rightEncoderB
};
