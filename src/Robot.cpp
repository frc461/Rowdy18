#include "WPILib.h"
#include "Robot.h"
#include "XboxJoystickMap.h"
#include "RateEncoder.h"
#include <Timer.h>

#define DEADZONE 0.1
#define LEFT_TOLERANCE 0.1
#define RIGHT_TOLERANCE 0.1

class Robot: public IterativeRobot
{

	Joystick leftStick; // only joystick
	Joystick rightStick;
	Joystick xbox;
	Victor frontLeft;	//motor controller
	Victor frontRight;
	Victor backLeft;
	Victor backRight;
	RobotDrive driveTrain;	//handles driving methods
	Spark intakeRoller;	//motor controller
	DoubleSolenoid intake;	//pneumatic controller
	Spark climberUp;
	DoubleSolenoid shifter;
	Spark leftShooter;
	Spark leftFeeder;
	Spark rightShooter;
	Spark rightFeeder;
	RateEncoder leftEncoder;	//sensor
	RateEncoder rightEncoder;
	PIDController leftPID;	//error adjustor
	PIDController rightPID;
	ADXRS450_Gyro gyro;
	Timer timer;

	int mode;
	int state;
	double initialAngle;

public:
	Robot():
		leftStick(0),	//get used to it
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
		shifter(shifterForwardPWM, shifterReversePWM),	//change gears
		leftShooter(leftShooterPWM),
		leftFeeder(leftFeederPWM),
		rightShooter(rightShooterPWM),
		rightFeeder(rightFeederPWM),
		leftEncoder(leftEncoderA, leftEncoderB),
		rightEncoder(rightEncoderA, rightEncoderB),
		leftPID(0, 0, 0, &leftEncoder, &leftShooter),
		rightPID(0, 0, 0, &rightEncoder, &rightShooter),
		timer()
	{
		SmartDashboard::init();
		//b = DriverStationLCD::GetInstance();
	}

private:
	void RobotInit()
	{
	}

	void backUpMod(double seconds) {
		driveTrain.TankDrive(-1, -1, false);
		if (timer.Get() > seconds) {
			driveTrain.TankDrive(0, 0.0, false);
			timer.Reset();
			state++;
		}
	}

	void autoRightGearHighGoal() {
		if (state == rGHG_LowGear) {
			shifter.Set(DoubleSolenoid::kForward);
			timer.Reset();
			state++;
		}

		else if (state == rGHG_BackUp0) {
			backUpMod(2);
		}

		else if (state == rGHG_RotateLeft) {
			if (initialAngle == -1) {
				initialAngle = gyro.GetAngle();
			}
			if ((int) (fabs(gyro.GetAngle() - initialAngle)) % 360 <= 85) {
				driveTrain.TankDrive(-0.5, 0.5);
			}
			else {
				driveTrain.TankDrive(0.0, 0);
				timer.Reset();
				state++;
			}
		}

		else if (state == rGHG_BackUp1) {
			backUpMod(2);
		}

		else if (state == rGHG_PlaceGear) {
			//TODO: implement later
			//note: back up & wait & forward
			if (timer.Get() > 4) {
				timer.Reset();
				state++;
			}
		}

		else if (state == rGHG_DriveForward) {
			driveTrain.TankDrive(1, 1, false);
			if (timer.Get() > 2) {
				driveTrain.TankDrive(0.0, 0, false);
				timer.Reset();
				state++;
			}
		}
		else if (state == rGHG_ShootFuel) {
			Shooting();
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		}
	}

	void autoRightGearHighGoalReload() {
		if (state <= rGHGR_RGHG) {
			autoRightGearHighGoal();
			if (state == rGHG_ShootFuel + 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == rGHGR_BackUp0) {
			backUpMod(2);
		}
		else if (state == rGHGR_LowerIntake) {
			intake.Set(DoubleSolenoid::kReverse);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == rGHGR_DriveToHopper) {
			//TODO: drive to hopper
			if (timer.Get() > 3) {
				timer.Reset();
				state++;
			}
		}
	}

	void autoCenterGear() {
		if (state == 0) {
			backUpMod(15);
		}
	}

	void autoLeftGearReload() {
		if (state == lGR_LowGear) {
			shifter.Set(DoubleSolenoid::kForward);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_BackUp0) {
			backUpMod(2);
		}
		else if (state == lGR_RotateRight0) {
			if (initialAngle == -1) {
				initialAngle = gyro.GetAngle();
			}
			if ((int) (fabs(gyro.GetAngle() - initialAngle)) % 360 <= 85) {
				driveTrain.TankDrive(0.5, -0.5);
			}
			else {
				driveTrain.TankDrive(0.0, 0);
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_BackUp1) {
			backUpMod(2);
		}
		else if (state == lGR_PlaceGear) {
			//TODO: implement later
			if (timer.Get() > 4) {
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_DriveForward) {
			driveTrain.TankDrive(1, 1, false);
			if (timer.Get() > 2) {
				driveTrain.TankDrive(0.0, 0, false);
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_RotateRight1) {
			if (initialAngle == -1) {
				initialAngle = gyro.GetAngle();
			}
			if ((int) (fabs(gyro.GetAngle() - initialAngle)) % 360 <= 85) {
				driveTrain.TankDrive(0.5, -0.5);
			}
			else {
				driveTrain.TankDrive(0.0, 0);
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_LowerIntake) {
			intake.Set(DoubleSolenoid::kReverse);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_DriveToHopper) {
			//TODO: drive to hopper
			if (timer.Get() > 3) {
				timer.Reset();
				state++;
			}
		}
	}

	void autoHighGoalReload() {

	}

	void AutonomousInit()
	{
		mode = 0;
		state = 0;
		initialAngle = -1;

		timer.Start();
	}

	void AutonomousPeriodic()
	{
		switch (mode) {
		case rightGearHighGoal:
			autoRightGearHighGoal();
			break;
		case rightGearHighGoalReload:
			autoRightGearHighGoalReload();
			break;
		case centerGear:
			autoCenterGear();
			break;
		case leftGearReload:
			autoLeftGearReload();
			break;
		case highGoalReload:
			autoHighGoalReload();
			break;
		default:
			std::cout << "you screwed up" << std::endl;
		}
	}

	void Shooting()
	{
		if (fabs(leftEncoder.GetRate() - 10) < LEFT_TOLERANCE) {
			leftFeeder.SetSpeed(0.5);
		}
		else {
			leftFeeder.SetSpeed(0);
		}
		if (fabs(rightEncoder.GetRate() - 10) < RIGHT_TOLERANCE) {
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

		driveTrain.TankDrive(left, right);	//assign driving method & args

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
