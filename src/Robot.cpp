#include <WPILib.h>
#include "Robot.h"
#include "XboxJoystickMap.h"
#include "RateEncoder.h"
#include <Timer.h>
#include "SettablePIDOut.h"

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

//#define D_SHOOTING
#define D_INTAKE
//#define D_CONVEYOR

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
  DoubleSolenoid shifter;
  Spark leftShooter;
  Spark leftTower;
  Spark rightShooter;
  Spark rightTower;
  RateEncoder leftShooterEncoder;      //sensor
  RateEncoder rightShooterEncoder;
  Encoder leftDriveEncoder;
  Encoder rightDriveEncoder;
  PIDController leftPID;        //error adjustor
  PIDController rightPID;
  ADXRS450_Gyro gyro;
  Timer timer;
  Spark conveyor;
  SettablePIDOut leftOut;
  SettablePIDOut rightOut;
  AnalogInput currentSensor;
  Preferences *prefs;
  int mode = 0;
  int state = 0;
  double initialAngle = -1;
  double shootingSpeed = SHOOTING_SPEED;
  PowerDistributionPanel *pdp = new PowerDistributionPanel();

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
    leftShooterEncoder(leftShooterEncoderA, leftShooterEncoderB),
    rightShooterEncoder(rightShooterEncoderA, rightShooterEncoderB),
	leftDriveEncoder(leftDriveEncoderA, leftDriveEncoderB),
	rightDriveEncoder(rightDriveEncoderA, rightDriveEncoderB),
    leftPID(0.1, 0.1, 0.1, &leftShooterEncoder, &leftOut),
    rightPID(0.1, 0.1, 0.1, &rightShooterEncoder, &rightOut),
    timer(),
    conveyor(conveyorPWM),
	currentSensor(3),
	prefs(){
    SmartDashboard::init();
    //b = DriverStationLCD::GetInstance();
  }

private:
  double rollerSpeed;
  void RobotInit() {
	    prefs = Preferences::GetInstance();
	    rollerSpeed = prefs->GetDouble("rollerSpeed", 0.9);
  }

  void backUpMod(double seconds) {
    driveTrain.TankDrive(-1, -1, false);
    if (timer.Get() > seconds) {
      driveTrain.TankDrive(0, 0.0, false);
      timer.Reset();
      state++;
    }
  }

#ifdef USE_PID_FOR_MANUAL_SHOOTING
  void Shoot() {
	  leftPID.Enable();
	  rightPID.Enable();

	  leftShooter.SetSpeed(leftOut.m_output);
	  rightShooter.SetSpeed(-rightOut.m_output);
  }
#else
  void Shoot() {
	  leftShooter.SetSpeed(shootingSpeed);
	  rightShooter.SetSpeed(-shootingSpeed);
  }

#endif

  void StopShooting() {
	  leftPID.Disable();
	  rightPID.Disable();
	  leftPID.Reset();
	  rightPID.Reset();
	  leftShooter.SetSpeed(0);
	  rightShooter.SetSpeed(0);
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

  void AutomaticShooting() {
    Shoot();

    conveyor.SetSpeed(CONVEYOR_SPEED);
    if (fabs(leftShooterEncoder.GetRate() - shootingSpeed) < LEFT_TOLERANCE) {
      leftTower.SetSpeed(TOWER_SPEED);
    } else {
      leftTower.SetSpeed(0);
    }

    if (fabs(rightShooterEncoder.GetRate() - shootingSpeed) < RIGHT_TOLERANCE) {
      rightTower.SetSpeed(-TOWER_SPEED);
    } else {
      rightTower.SetSpeed(0);
    }
  }

  void TeleopInit() {
    leftPID.SetSetpoint(shootingSpeed);
    rightPID.SetSetpoint(shootingSpeed);
  }

  void ManualShooting() {
#ifdef D_SHOOTING
    printf("Shooting manually\n");
#endif
    leftTower.SetSpeed(TOWER_SPEED);
    rightTower.SetSpeed(-TOWER_SPEED);
    conveyor.SetSpeed(CONVEYOR_SPEED);

    Shoot();
  }

  // Takes a speed [-1.0, 1.0] and scales to [0.0, 1.0]
  double ScaledShootingSpeed(double rawAxis) {
    return (rawAxis/2) + 0.5;
  }

  void Monitor() {
	  SmartDashboard::PutNumber("Total current", pdp->GetTotalCurrent());
	  SmartDashboard::PutNumber("Conveyor current", pdp->GetCurrent(pdpConveyor));
	  SmartDashboard::PutNumber("Intake current", pdp->GetCurrent(pdpIntake));
	  SmartDashboard::PutNumber("Intake current analog", (currentSensor.GetAverageVoltage() - 0.6) / 0.04);

	  SmartDashboard::PutNumber("Left Drive Encoder", leftDriveEncoder.Get());
	  SmartDashboard::PutNumber("Right Drive Encoder", rightDriveEncoder.Get());
	  SmartDashboard::PutNumber("Left Shooter Encoder", leftShooterEncoder.Get());
	  SmartDashboard::PutNumber("Right Shooter Encoder", rightShooterEncoder.Get());

	  SmartDashboard::PutNumber("Shooting setpoint", shootingSpeed * 6000);
  }

  void TeleopPeriodic() {
    // TODO: Check this axis
    double left = driveControl.GetRawAxis(XboxAxisLeftStickY);
    double right = driveControl.GetRawAxis(XboxAxisRightStickY);

    if (fabs(left) < DEADZONE) {
      left = 0;
    }
    if (fabs(right) < DEADZONE) {
      right = 0;
    }

    driveTrain.TankDrive(left, right);  //assign driving method & args

    if (op.GetRawButton(shootingModeSwitch)) {
#ifdef D_SHOOTING
      printf("Manual shooting\n");
#endif
      shootingSpeed = -ScaledShootingSpeed(op.GetRawAxis(changeShooterSpeed));
    } else {
    	shootingSpeed = SHOOTING_SPEED;
    }

    leftPID.SetSetpoint(shootingSpeed);
    rightPID.SetSetpoint(-shootingSpeed);

    if (op.GetRawButton(intakePositionSwitch)) {
      intake.Set(INTAKE_DOWN);
    }
    else {
      intake.Set(INTAKE_UP);
    }

    if (op.GetRawButton(spinIntakeForwardButton)) {
#ifdef D_INTAKE
    	printf("Moving intake forward\n");
#endif
      intakeRoller.Set(rollerSpeed);
    } else if (op.GetRawButton(spinIntakeBackwardButton)) {
#ifdef D_INTAKE
    	printf("Moving intake backward\n");
#endif
      intakeRoller.Set(-rollerSpeed);
    } else {
#ifdef D_INTAKE
    	printf("Stopping intake\n");
#endif
      intakeRoller.Set(0);
    }

    if (op.GetRawButton(climberButton)) {
      climber.SetSpeed(CLIMBER_SPEED);
    }
    else {
      climber.SetSpeed(0);
    }


    if (driveControl.GetRawAxis(shiftGearsAxis) > 0.5) {
      shifter.Set(SHIFTER_HIGH);
    }
    else {
      shifter.Set(SHIFTER_LOW);
    }

    if (!op.GetRawButton(shootingModeSwitch)) {
      //Automatic mode
      if (op.GetRawButton(shootingButton)) {
#ifdef D_SHOOTING
        printf("Shooting button pressed\n");
#endif
        AutomaticShooting();
      } else {
        StopShooting();
      }
    } else {
      StopShooting();
      if (op.GetRawButton(shootingTowersConveyorButton)) {
#ifdef D_SHOOTING
        printf("Manual shooting button pressed\n");
#endif
        ManualShooting();
      }
      else {
        if (op.GetRawButton(conveyorIn)) {
#ifdef D_CONVEYOR
          printf("Conveyor in pressed\n");
#endif
          conveyor.SetSpeed(CONVEYOR_SPEED); //neg or pos
        } else if (op.GetRawButton(conveyorOut)) {
#ifdef D_CONVEYOR
          printf("Conveyor out pressed\n");
#endif
          conveyor.SetSpeed(-CONVEYOR_SPEED);
        } else {
#ifdef D_CONVEYOR
          printf("Stopping conveyor\n");
#endif
          conveyor.SetSpeed(0.0);
        }

        if (op.GetRawButton(shootingButton)){
#ifdef D_SHOOTING
          printf("Just manual shooting\n");
          printf("Shooting speed: %lf\n", shootingSpeed);
#endif
          Shoot();
        } else {
#ifdef D_SHOOTING
          printf("Stopping manual shooting\n");
#endif
          StopShooting();
        }

        if (op.GetRawButton(towersInButton)) {
          leftTower.SetSpeed(TOWER_SPEED); //neg or pos
          rightTower.SetSpeed(-TOWER_SPEED);
        } else if (op.GetRawButton(towersOutButton)) {
          leftTower.SetSpeed(-TOWER_SPEED);
          rightTower.SetSpeed(TOWER_SPEED);
        } else {
          leftTower.SetSpeed(0.0);
          rightTower.SetSpeed(0.0);
        }
      }
    }

    Monitor();
  }

  void TestPeriodic() {
  }

};

START_ROBOT_CLASS(Robot);
