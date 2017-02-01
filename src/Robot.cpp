#include "WPILib.h"
#include "Robot.h"
#include "XboxJoystickMap.h"
#include "RateEncoder.h"
#include <Timer.h>

#define DEADZONE 0.1
#define LEFT_TOLERANCE 0.1
#define RIGHT_TOLERANCE 0.1

#define INTAKE_DOWN DoubleSolenoid::kReverse
#define INTAKE_UP DoubleSolenoid::kForward
#define SHIFTER_LOW DoubleSolenoid::kForward
#define SHIFTER_HIGH DoubleSolenoid::kReverse

class Robot: public IterativeRobot {

	Joystick driveControl;
	Joystick xbox;
	Victor frontLeft;	//motor controller
	Victor frontRight;
	Victor backLeft;
	Victor backRight;
	RobotDrive driveTrain;	//handles driving methods
	Spark intakeRoller;	//motor controller
	DoubleSolenoid intake;	//pneumatic controller
	Spark climber;
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

	int mode = 0;
	int state = 0;
	double initialAngle = -1;

public:
	Robot() :
			driveControl(0), xbox(2), frontLeft(frontLeftPWM), frontRight(
					frontRightPWM), backLeft(backLeftPWM), backRight(
					backRightPWM), driveTrain(frontLeft, backLeft, frontRight,
					backRight), intakeRoller(intakeRollerPWM), intake(
					intakeForwardPCM, intakeReversePCM), climber(climberPWM), shifter(
					shifterForwardPCM, shifterReversePCM),	//change gears
			leftShooter(leftShooterPWM), leftFeeder(leftFeederPWM), rightShooter(
					rightShooterPWM), rightFeeder(rightFeederPWM), leftEncoder(
					leftEncoderA, leftEncoderB), rightEncoder(rightEncoderA,
					rightEncoderB), leftPID(0, 0, 0, &leftEncoder,
					&leftShooter), rightPID(0, 0, 0, &rightEncoder,
					&rightShooter), timer() {
		SmartDashboard::init();
		//b = DriverStationLCD::GetInstance();
	}

private:
	void RobotInit() {
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
			} else {
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
		} else if (state == rGHG_ShootFuel) {
			Shooting();
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		} else {
			StopShooting();
		}
	}

	void autoRightGearHighGoalReload() {
		if (state <= rGHGR_RGHG) {
			autoRightGearHighGoal();
		} else if (state == rGHGR_BackUp0) {
			backUpMod(2);
		} else if (state == rGHGR_LowerIntake) {
			intake.Set(INTAKE_DOWN);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == rGHGR_DriveToHopper) {
			driveTrain.TankDrive(0.5, 0.5);
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
			shifter.Set(SHIFTER_LOW);
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
			intake.Set(INTAKE_DOWN);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == lGR_DriveToHopper) {
			driveTrain.TankDrive(0.5, 0.5);
			if (timer.Get() > 3) {
				timer.Reset();
				state++;
			}
		}
	}

	void autoHighGoalReload() {
		if (state == HGR_DriveForward0) {
			driveTrain.TankDrive(1, 1, false);
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_RotateRight0) {
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
		else if (state == HGR_DriveForward1) {
			driveTrain.TankDrive(1, 1, false);
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_RotateRight1) {
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
		else if (state == HGR_LowerIntake) {
			intake.Set(INTAKE_DOWN);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_RunIntake) {
			intakeRoller.SetSpeed(.5);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_DriveBackwards) {
			driveTrain.TankDrive(-0.5, -0.5, false);
			if (timer.Get() > 1) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_RotateRight2) {
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
		else if (state == HGR_DriveForward) {
			driveTrain.TankDrive(1, 1, false);
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		}
		else if (state == HGR_Shoot) {
			Shooting();
			if (timer.Get() > 5) {
				timer.Reset();
				state++;
			}
		} else {
			StopShooting();
		}
	}

	void AutonomousInit() {
		mode = 0;
		state = 0;
		initialAngle = -1;

		timer.Start();
	}

	void AutonomousPeriodic() {
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

	void Shooting() {
		leftPID.Enable();
		rightPID.Enable();
		if (fabs(leftEncoder.GetRate() - 10) < LEFT_TOLERANCE) {
			leftFeeder.SetSpeed(0.5);
		} else {
			leftFeeder.SetSpeed(0);
		}

		if (fabs(rightEncoder.GetRate() - 10) < RIGHT_TOLERANCE) {
			rightFeeder.SetSpeed(0.5);
		} else {
			rightFeeder.SetSpeed(0);
		}
	}

	void StopShooting() {
		leftPID.Disable();
		rightPID.Disable();
	}

	void TeleopInit() {
		leftPID.SetSetpoint(10);
		rightPID.SetSetpoint(10);
	}

	void TeleopPeriodic() {
// TODO: Check this axis
		double left = driveControl.GetRawAxis(1);
		double right = driveControl.GetRawAxis(5);

		if (fabs(left) < DEADZONE) {
			left = 0;
		}
		if (fabs(right) < DEADZONE) {
			right = 0;
		}

		driveTrain.TankDrive(left, right);	//assign driving method & args

		if (xbox.GetRawButton(lowerIntakeButton)) {
			intake.Set(INTAKE_DOWN);
		}
		else if (xbox.GetRawButton(raiseIntakeButton)) {
			intake.Set(INTAKE_UP);
		}
		else {
			intake.Set(DoubleSolenoid::kOff);
		}

		if (xbox.GetRawButton(spinIntakeForwardButton)) {
			intakeRoller.SetSpeed(.5);
		} else if (xbox.GetRawButton(spinIntakeBackwardButton)) {
			intakeRoller.SetSpeed(-.5);
		} else {
			intakeRoller.SetSpeed(0);
		}

		if (xbox.GetRawButton(climberButton)) {
			climber.SetSpeed(.5);
		}
		else {
			climber.SetSpeed(0);
		}


		if (xbox.GetRawButton(shiftGearsButton)) {
			shifter.Set(SHIFTER_HIGH);
		}
		else {
			shifter.Set(SHIFTER_LOW);
		}

		if (xbox.GetRawAxis(shootingButton) > .5) {
			Shooting();
		} else {
			StopShooting();
			leftShooter.SetSpeed(0);
			leftFeeder.SetSpeed(0);
			rightShooter.SetSpeed(0);
			rightFeeder.SetSpeed(0);
		}
	}

	void TestPeriodic() {
	}

};

START_ROBOT_CLASS(Robot);
