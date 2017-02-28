#include <WPILib.h>
#include "Robot.h"
#include "XboxJoystickMap.h"
#include <Timer.h>
#include "SettablePIDOut.h"
#include "RateCounter.h"
#include "Logger.h"

#define DEADZONE 0.1
#define LEFT_TOLERANCE 0.1
#define RIGHT_TOLERANCE 0.1

#define TOWER_SPEED -0.9
#define SHOOTING_SPEED 0.8
#define CONVEYOR_SPEED 0.9
#define ROLLER_SPEED 0.9
#define CLIMBER_SPEED -0.9

#define INTAKE_DOWN DoubleSolenoid::kReverse
#define INTAKE_UP DoubleSolenoid::kForward
#define SHIFTER_LOW DoubleSolenoid::kReverse
#define SHIFTER_HIGH DoubleSolenoid::kForward

#define MAX_RPM -6000
//#define USE_PID_FOR_MANUAL_SHOOTING

class Robot: public IterativeRobot {

  Joystick driveControl;
  Joystick op;
  Victor frontLeft;     //motor controller
  Victor frontRight;
  Victor backLeft;
  Victor backRight;
  RobotDrive driveTrain;        //handles driving methods
  Spark intakeRoller;   //motor controller
  DoubleSolenoid intake;        //pneumatic controller
  Spark climber;
  bool useClimberBackwards = false;
  SendableChooser<bool> *climbChooser = new SendableChooser<bool>();
  DoubleSolenoid shifter;
  Spark leftShooter;
  Spark leftTower;
  Spark rightShooter;
  Spark rightTower;
  RateCounter leftShooterEncoder;      //sensor
  RateCounter rightShooterEncoder;
  Encoder leftDriveEncoder;
  Encoder rightDriveEncoder;
  SettablePIDOut leftOut;
  SettablePIDOut rightOut;
  PIDController leftPID;        //error adjustor
  PIDController rightPID;
  ADXRS450_Gyro gyro;
  Timer timer;
  Spark conveyor;
  AnalogInput currentSensor;
  Preferences *prefs;
  int mode = 0;
  int state = 0;
  double initialAngle = -1;
  double shootingSpeed = SHOOTING_SPEED;
  PowerDistributionPanel *pdp = new PowerDistributionPanel();
  Logger *logger = new Logger();
  Joystick leftJoystick;
  Joystick rightJoystick;
  bool useXboxControllerForDriving = true;
  SendableChooser<bool> *driveChooser = new SendableChooser<bool>();
  double driveAngle = -1;

public:
  Robot() :
    driveControl(0),
    op(1),
    frontLeft(frontLeftPWM),
    frontRight(frontRightPWM),
    backLeft(backLeftPWM),
    backRight(backRightPWM),
    driveTrain(frontLeft, backLeft, frontRight, backRight),
    intakeRoller(intakeRollerPWM),
    intake(intakeForwardPCM, intakeReversePCM),
    climber(climberPWM),
    shifter(shifterForwardPCM, shifterReversePCM),   //change gears
    leftShooter(leftShooterPWM),
    leftTower(leftTowerPWM),
    rightShooter(rightShooterPWM),
    rightTower(rightTowerPWM),
    leftShooterEncoder(leftShooterEncoderA),
    rightShooterEncoder(rightShooterEncoderA),
	leftDriveEncoder(leftDriveEncoderA, leftDriveEncoderB),
	rightDriveEncoder(rightDriveEncoderA, rightDriveEncoderB),
	leftOut(),
	rightOut(),
    leftPID(0, 0.00001, 0, &leftShooterEncoder, &leftOut,0.05),
    rightPID(0, 0.00001, 0, &rightShooterEncoder, &rightOut,0.05),
    timer(),
    conveyor(conveyorPWM),
	currentSensor(3),
	prefs(),
	leftJoystick(4),
	rightJoystick(5)

  {
    SmartDashboard::init();
    //b = DriverStationLCD::GetInstance();
  }

private:
  double rollerSpeed;
  void RobotInit() {
	    prefs = Preferences::GetInstance();
	    rollerSpeed = prefs->GetDouble("rollerSpeed", 0.9);
	    driveChooser->AddDefault("Xbox controller", true);
	    driveChooser->AddObject("Joysticks", false);
	    climbChooser->AddDefault("Forward", false);
	    climbChooser->AddObject("Backward", true);
	    SmartDashboard::PutData("Drive Control", driveChooser);
	    SmartDashboard::PutData("Climber Direction", climbChooser)
  }

  void backUpMod(double seconds) {
    driveTrain.TankDrive(-1, -1, false);
    if (timer.Get() > seconds) {
      driveTrain.TankDrive(0, 0.0, false);
      timer.Reset();
      state++;
    }
  }

  void DriveStraight(double speed) {
	  if (driveAngle == -1) {
		  driveAngle = gyro.GetAngle() % 360;
	  }

	  double currentAngle = gyro.GetAngle() % 360;
	  if (currentAngle > 180) {
		  currentAngle -= 360;
	  }

	  double error = currentAngle - driveAngle;
	  double correction = (error / 180) / 2;

	  driveTrain.TankDrive(speed + correction, speed - correction);

	  logger->Log(logDriveTrain, "Current angle %lf, target angle: %lf, correction: %lf\n", currentAngle, driveAngle, correction);
  }

  void DisabledInit() {
	  logger->CloseLog();
  }

#ifdef USE_PID_FOR_MANUAL_SHOOTING
  void Shoot() {
	  if (!leftPID.IsEnabled()) {
		  leftPID.Enable();
	  }
	  if (!rightPID.IsEnabled()) {
		  rightPID.Enable();
	  }

	  double outLeft = -(leftOut.m_output + leftPID.GetSetpoint() * .00003);
	  double outRight = rightOut.m_output + rightPID.GetSetpoint() * .00003;

	  leftShooter.SetSpeed(outLeft);
	  rightShooter.SetSpeed(outRight);
#ifdef D_SHOOTING
	  printf("left input: %lf, setpoint: %lf, output: %lf, error: %lf, avg error: %lf\n", leftShooterEncoder.PIDGet(), leftPID.GetSetpoint(), outLeft, leftPID.GetError(), leftPID.GetAvgError());
	  printf("right input: %lf, setpoint: %lf, output: %lf, error: %lf, avg error: %lf\n", rightShooterEncoder.PIDGet(), rightPID.GetSetpoint(), outRight, rightPID.GetError(), rightPID.GetAvgError());
	  printf("LeftP: %lf, RightP: %lf\n", leftPID.GetP(), rightPID.GetP());
#endif
  }
#else
  void Shoot() {
	  leftShooter.SetSpeed(shootingSpeed);
	  rightShooter.SetSpeed(-shootingSpeed);
	  logger->Log(logShooter, "Running shooter at %lf\n", shootingSpeed);
  }

#endif

  void StopShooting() {
	  leftPID.Disable();
	  rightPID.Disable();
	  leftPID.Reset();
	  rightPID.Reset();
	  leftShooter.SetSpeed(0);
	  rightShooter.SetSpeed(0);
	  logger->Log(logShooter, "Stopping shooter\n");
  }


  void autoRightGearHighGoal() {
    if (state == rGHG_LowGear) {
      shifter.Set(DoubleSolenoid::kForward);
      timer.Reset();
      state++;
    } else if (state == rGHG_BackUp0) {
      backUpMod(2);
    } else if (state == rGHG_RotateLeft) {
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
    } else if (state == rGHG_BackUp1) {
      backUpMod(2);
    } else if (state == rGHG_PlaceGear) {
      //TODO: implement later
      //note: back up & wait & forward
      if (timer.Get() > 4) {
        timer.Reset();
        state++;
      }
    } else if (state == rGHG_DriveForward) {
      driveTrain.TankDrive(1, 1, false);
      if (timer.Get() > 2) {
        driveTrain.TankDrive(0.0, 0, false);
        timer.Reset();
        state++;
      }
    } else if (state == rGHG_ShootFuel) {
      AutomaticShooting();
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
      AutomaticShooting();
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
    logger->OpenNewLog("_auton");
    logger->Log(logAuton, "Running auton %d\n", mode);
  }

  void AutonomousPeriodic() {
	  logger->Log(logAuton, "Auton is in state %d\n", state);
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
      logger->Log(logAuton, "Ran invalid auton\n");
    }
  }

  void AutomaticShooting() {
	  logger->Log(logShooter, "Automatic shooting\n");
    Shoot();

    conveyor.SetSpeed(CONVEYOR_SPEED);
    if (fabs(leftShooterEncoder.GetPeriod() - shootingSpeed) < LEFT_TOLERANCE) { //change to fit new encoders
      leftTower.SetSpeed(TOWER_SPEED);
      logger->Log(logShooter, "Moving left tower\n");
    } else {
    	logger->Log(logShooter, "Stopping left tower\n");
      leftTower.SetSpeed(0);
    }

    if (fabs(rightShooterEncoder.GetPeriod() - shootingSpeed) < RIGHT_TOLERANCE) { //change to fit new encoders
      rightTower.SetSpeed(-TOWER_SPEED);
      logger->Log(logShooter, "Moving right tower\n");
    } else {
    	logger->Log(logShooter, "Stopping right tower\n");
      rightTower.SetSpeed(0);
    }
  }

  void TeleopInit() {
    leftPID.SetSetpoint(shootingSpeed * MAX_RPM);
    rightPID.SetSetpoint(shootingSpeed * MAX_RPM);
//    leftPID.SetOutputRange(0, .6);
//    rightPID.SetOutputRange(0, 0.6);
    logger->OpenNewLog("_teleop");
    useXboxControllerForDriving = driveChooser->GetSelected();
    useClimberBackwards = climbChooser->GetSelected();
  }

  void ManualShooting() {
    logger->Log(logShooter, "Shooting manually\n");
    leftTower.SetSpeed(TOWER_SPEED);
    rightTower.SetSpeed(-TOWER_SPEED);
    conveyor.SetSpeed(CONVEYOR_SPEED);
    Shoot();
  }

  // Takes a speed [-1.0, 1.0] and scales to [0.0, 1.0]
  double ScaledShootingSpeed(double rawAxis) {
    return (-rawAxis/2) + 0.5;
  }

  void Monitor() {
	  SmartDashboard::PutNumber("Total current", pdp->GetTotalCurrent());
	  SmartDashboard::PutNumber("Conveyor current", pdp->GetCurrent(pdpConveyor));
	  SmartDashboard::PutNumber("Intake current", pdp->GetCurrent(pdpIntake));
	  SmartDashboard::PutNumber("Intake current analog", (currentSensor.GetAverageVoltage() - 0.6) / 0.04);

	  SmartDashboard::PutNumber("Left Drive Encoder", leftDriveEncoder.Get());
	  SmartDashboard::PutNumber("Right Drive Encoder", rightDriveEncoder.Get());
	  SmartDashboard::PutNumber("Left Shooter Encoder", leftShooterEncoder.PIDGet());
	  SmartDashboard::PutNumber("Right Shooter Encoder", rightShooterEncoder.PIDGet());

	  SmartDashboard::PutNumber("Shooting setpoint", shootingSpeed * MAX_RPM);
  }

  void TeleopPeriodic() {
	double left, right;
	bool driveStraight;
	if (useXboxControllerForDriving) {
	  left = driveControl.GetRawAxis(XboxAxisLeftStickY);
      right = driveControl.GetRawAxis(XboxAxisRightStickY);
	} else {
		left = leftJoystick.GetRawAxis(1);
		right = rightJoystick.GetRawAxis(1);
		driveStraight = leftJoystick.GetRawButton(driveStraightButtonLeftJoystick);
	}
    logger->Log(logDriveTrain, "Read from joysticks (%lf, %lf)\n", left, right);
    if (fabs(left) < DEADZONE) {
      left = 0;
    }
    if (fabs(right) < DEADZONE) {
      right = 0;
    }

    if (driveStraight) {
    	DriveStraight(left > right ? left : right);
    } else {
    	driveTrain.TankDrive(left, right);  //assign driving method & args
    	driveAngle = -1;
    }
    logger->Log(logDriveTrain, "Driving at (%lf, %lf)\n", left, right);

    if (op.GetRawButton(shootingModeSwitch)) {
      logger->Log(logShooter, "Manual shooting mode\n");
      shootingSpeed = -ScaledShootingSpeed(op.GetRawAxis(changeShooterSpeed));
    } else {
    	shootingSpeed = SHOOTING_SPEED;
    }

    double setpoint = shootingSpeed * MAX_RPM;
    leftPID.SetSetpoint(setpoint);
    rightPID.SetSetpoint(setpoint);
    logger->Log(logShooter, "Current pid setpoint: %lf\n", setpoint);

    if (op.GetRawButton(intakePositionSwitch)) {
      intake.Set(INTAKE_DOWN);
      logger->Log(logIntake, "Moving intake down\n");
    }
    else {
    	logger->Log(logIntake, "Moving intake up\n");
      intake.Set(INTAKE_UP);
    }

    if (op.GetRawButton(spinIntakeForwardButton)) {
    	logger->Log(logIntake, "Spinning intake forward\n");
      intakeRoller.Set(rollerSpeed);
    } else if (op.GetRawButton(spinIntakeBackwardButton)) {
    	logger->Log(logIntake, "Spinning intake backward\n");
      intakeRoller.Set(-rollerSpeed);
    } else {
    	logger->Log(logIntake, "Stopping intake\n");
      intakeRoller.Set(0);
    }

    if (op.GetRawButton(climberButton)) {
    	logger->Log(logClimber, "Moving climber\n");
    	if(useClimberBackwards){
    		climber.SetSpeed(-CLIMBER_SPEED);
    	} else {
    		climber.SetSpeed(CLIMBER_SPEED);
    	}
    }
    else {
    	logger->Log(logClimber, "Stopping climber\n");
      climber.SetSpeed(0);
    }

    if (useXboxControllerForDriving) {
		if (driveControl.GetRawAxis(shiftGearsAxisXbox) > 0.5) {
		  shifter.Set(SHIFTER_HIGH);
		  logger->Log(logShifter, "High gear\n");
		}
		else {
			logger->Log(logShifter, "Low gear\n");
		  shifter.Set(SHIFTER_LOW);
		}
    } else {
    	if (rightJoystick.GetRawAxis(shiftGearsButtonRightJoystick)) {
    		shifter.Set(SHIFTER_HIGH);
    		logger->Log(logShifter, "High gear\n");
    	} else {
    		logger->Log(logShifter, "Low gear\n");
    		shifter.Set(SHIFTER_LOW);
    	}
    }

    if (!op.GetRawButton(shootingModeSwitch)) {
      //Automatic mode
      if (op.GetRawButton(shootingButton)) {
    	  logger->Log(logShooter, "Shooting button pressed\n");
        AutomaticShooting();
      } else {
        StopShooting();
      }
    } else {
      if (op.GetRawButton(shootingTowersConveyorButton)) {
        logger->Log(logShooter, "Manual shooting button pressed\n");
        ManualShooting();
      }
      else {
        if (op.GetRawButton(conveyorIn)) {
          logger->Log(logConveyor, "Conveyor in pressed\n");
          conveyor.SetSpeed(CONVEYOR_SPEED); //neg or pos
        } else if (op.GetRawButton(conveyorOut)) {
          logger->Log(logConveyor, "Conveyor out pressed\n");
          conveyor.SetSpeed(-CONVEYOR_SPEED);
        } else {
          logger->Log(logConveyor, "Stopping conveyor\n");
          conveyor.SetSpeed(0.0);
        }

        if (op.GetRawButton(shootingButton)){
          logger->Log(logShooter, "Just manual shooting\n");
          Shoot();
        } else {
          StopShooting();
        }

        if (op.GetRawButton(towersInButton)) {
          leftTower.SetSpeed(TOWER_SPEED); //neg or pos
          rightTower.SetSpeed(-TOWER_SPEED);
          logger->Log(logTower, "Moving towers up\n");
        } else if (op.GetRawButton(towersOutButton)) {
          leftTower.SetSpeed(-TOWER_SPEED);
          rightTower.SetSpeed(TOWER_SPEED);
          logger->Log(logTower, "Moving towers down\n");
        } else {
          leftTower.SetSpeed(0.0);
          rightTower.SetSpeed(0.0);
          logger->Log(logTower, "Stopping towers\n");
        }
      }
    }

    Monitor();
  }

  void TestPeriodic() {
  }

};

START_ROBOT_CLASS(Robot);
