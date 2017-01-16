#include "WPILib.h"
#include "Robot.h"
#include "XboxJoystickMap.h"
#include "RateEncoder.h"

#define DEADZONE 0.1
#define LEFT_TOLERANCE 0.1
#define RIGHT_TOLERANCE 0.1

class Robot: public IterativeRobot
{

	Joystick leftStick; // only joystick
	Joystick rightStick;
	Joystick xbox;
	Victor frontLeft;
	Victor frontRight;
	Victor backLeft;
	Victor backRight;
	RobotDrive driveTrain;
	Spark intakeRoller;
	DoubleSolenoid intake;
	Spark climberUp;
	DoubleSolenoid shifter;
	Spark leftShooter;
	Spark leftFeeder;
	Spark rightShooter;
	Spark rightFeeder;
	RateEncoder leftEncoder;
	RateEncoder rightEncoder;
	PIDController leftPID;
	PIDController rightPID;

public:
	Robot():
		leftStick(0),
		rightStick(1),
		xbox(2),
		frontLeft(frontLeftPWM),
		frontRight(frontRightPWM),
		backLeft(backLeftPWM),
		backRight(backRightPWM),
		driveTrain(frontLeft, backLeft, frontRight, backRight),
		intakeRoller(intakeRollerPWM),
		intake(intakeForwardPWM, intakeReversePWM),
		climberUp(climberUpPWM),
		shifter(shifterForwardPWM, shifterReversePWM),
		leftShooter(leftShooterPWM),
		leftFeeder(leftFeederPWM),
		rightShooter(rightShooterPWM),
		rightFeeder(rightFeederPWM),
		leftEncoder(leftEncoderA, leftEncoderB),
		rightEncoder(rightEncoderA, rightEncoderB),
		leftPID(0, 0, 0, &leftEncoder, &leftShooter),
		rightPID(0, 0, 0, &rightEncoder, &rightShooter)
	{
		SmartDashboard::init();
		//b = DriverStationLCD::GetInstance();
	}

private:
	void RobotInit()
	{
	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{
	}

	void Shooting()
	{
		if (leftEncoder.GetRate() > 10 - LEFT_TOLERANCE && leftEncoder.GetRate() < 10 + LEFT_TOLERANCE) {
			leftFeeder.SetSpeed(0.5);
		}
		else {
			leftFeeder.SetSpeed(0);
		}
		if (rightEncoder.GetRate() > 10 - RIGHT_TOLERANCE && rightEncoder.GetRate() < 10 + RIGHT_TOLERANCE) {
			rightFeeder.SetSpeed(0.5);
		}
		else {
			rightFeeder.SetSpeed(0);
		}
	}


	void TeleopInit()
	{
		leftPID.SetSetpoint(10);
		leftPID.Enable();
		rightPID.SetSetpoint(10);
		rightPID.Enable();
	}

	void TeleopPeriodic()
	{
		// TODO: Check this axis
		double left = leftStick.GetRawAxis(0);
		double right = rightStick.GetRawAxis(0);

		if (fabs(left) < DEADZONE) {
			left = 0;
		}
		if (fabs(right) < DEADZONE) {
			right = 0;
		}

		driveTrain.TankDrive(left, right);

		if (xbox.GetRawButton(lowerIntakeButton)) {
			intake.Set(DoubleSolenoid::kReverse);
		}
		else {
			intake.Set(DoubleSolenoid::kOff);
		}

		if(xbox.GetRawButton(raiseIntakeButton)) {
			intake.Set(DoubleSolenoid::kForward);
		}
		else {
			intake.Set(DoubleSolenoid::kOff);
		}
		if(xbox.GetRawButton(spinIntakeForwardButton)) {
			intakeRoller.SetSpeed(.5);
		}
		else {
			intakeRoller.SetSpeed(0);
		}
		if(xbox.GetRawButton(spinIntakeBackwardButton)){
			intakeRoller.SetSpeed(-.5);
		}
		else{
			intakeRoller.SetSpeed(0);
		}
		if(xbox.GetRawButton(climberUpButton)) {
			climberUp.SetSpeed(.5);
		}
		else{
			climberUp.SetSpeed(0);
		}
		if(rightStick.GetRawButton(shiftGearsButton)){
			shifter.Set(DoubleSolenoid::kForward);
		}
		else{
			shifter.Set(DoubleSolenoid::kReverse);
		}
		if(xbox.GetRawAxis(shootingButton) > .5) {
			Shooting();
		}
		else {
			leftShooter.SetSpeed(0);
			leftFeeder.SetSpeed(0);
			rightShooter.SetSpeed(0);
			rightFeeder.SetSpeed(0);
		}
	}

	void TestPeriodic()
	{
	}

};

START_ROBOT_CLASS(Robot);
