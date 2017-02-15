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
#define CLIMBER_SPEED 0.9

#define INTAKE_DOWN DoubleSolenoid::kReverse
#define INTAKE_UP DoubleSolenoid::kForward
#define SHIFTER_LOW DoubleSolenoid::kForward
#define SHIFTER_HIGH DoubleSolenoid::kReverse

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
  RateEncoder leftEncoder;      //sensor
  RateEncoder rightEncoder;
  PIDController leftPID;        //error adjustor
  PIDController rightPID;
  ADXRS450_Gyro gyro;
  Timer timer;
  Spark conveyor;
  SettablePIDOut leftOut;
  SettablePIDOut rightOut;
  int mode = 0;
  int state = 0;
  double initialAngle = -1;
  double shootingSpeed = SHOOTING_SPEED;

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
    leftEncoder(leftEncoderA, leftEncoderB),
    rightEncoder(rightEncoderA,rightEncoderB),
    leftPID(0.1, 0.1, 0.1, &leftEncoder, &leftOut),
    rightPID(0.1, 0.1, 0.1, &rightEncoder, &rightOut),
    timer(),
    conveyor(conveyorPWM) {
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
    conveyor.SetSpeed(CONVEYOR_SPEED);
    if (fabs(leftEncoder.GetRate() - shootingSpeed) < LEFT_TOLERANCE) {
      leftTower.SetSpeed(TOWER_SPEED);
    } else {
      leftTower.SetSpeed(0);
    }

    if (fabs(rightEncoder.GetRate() - shootingSpeed) < RIGHT_TOLERANCE) {
      rightTower.SetSpeed(-TOWER_SPEED);
    } else {
      rightTower.SetSpeed(0);
    }

    leftShooter.SetSpeed(leftOut.m_output);
    rightShooter.SetSpeed(-rightOut.m_output);
  }

  void StopShooting() {
    leftPID.Disable();
    rightPID.Disable();
  }

  void TeleopInit() {
    leftPID.SetSetpoint(shootingSpeed);
    rightPID.SetSetpoint(shootingSpeed);
  }

  void ManualShooting() {
    printf("Shooting manually\n");
    leftTower.SetSpeed(TOWER_SPEED);
    rightTower.SetSpeed(-TOWER_SPEED);
    conveyor.SetSpeed(CONVEYOR_SPEED);
    leftShooter.SetSpeed(shootingSpeed);
    rightShooter.SetSpeed(-shootingSpeed);
  }

  // Takes a speed [-1.0, 1.0] and scales to [0.0, 1.0]
  double ScaledShootingSpeed(double rawAxis) {
    return (rawAxis/2) + 0.5;
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
      printf("Manual shooting\n");
      shootingSpeed = -ScaledShootingSpeed(op.GetRawAxis(changeShooterSpeed));
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
      intakeRoller.SetSpeed(ROLLER_SPEED);
    } else if (op.GetRawButton(spinIntakeBackwardButton)) {
      intakeRoller.SetSpeed(-ROLLER_SPEED);
    } else {
      intakeRoller.SetSpeed(0);
    }

    if (op.GetRawButton(climberButton)) {
      climber.SetSpeed(CLIMBER_SPEED);
    }
    else {
      climber.SetSpeed(0);
    }


    if (op.GetRawButton(shiftGearsButton)) {
      shifter.Set(SHIFTER_HIGH);
    }
    else {
      shifter.Set(SHIFTER_LOW);
    }

    if (!op.GetRawButton(shootingModeSwitch)) {
      //Automatic mode
      if (op.GetRawButton(shootingButton)) {
        printf("Shooting button pressed\n");
        Shooting();
      } else {
        StopShooting();
      }
    } else {
      StopShooting();
      if (op.GetRawButton(shootingTowersConveyorButton)) {
        printf("Manual shooting button pressed\n");
        ManualShooting();
      }
      else {
        if (op.GetRawButton(conveyorIn)) {
          printf("Conveyor in pressed\n");
          conveyor.SetSpeed(CONVEYOR_SPEED); //neg or pos
        } else if (op.GetRawButton(conveyorOut)) {
          printf("Conveyor out pressed\n");
          conveyor.SetSpeed(-CONVEYOR_SPEED);
        } else {
          printf("Stopping conveyor\n");
          conveyor.SetSpeed(0.0);
        }

        if (op.GetRawButton(shootingButton)){
          printf("Just manual shooting\n");
          printf("Shooting speed: %lf\n", shootingSpeed);
          leftShooter.SetSpeed(shootingSpeed);
          rightShooter.SetSpeed(-shootingSpeed);
        } else {
          printf("Stopping manual shooting\n");
          leftShooter.SetSpeed(0);
          rightShooter.SetSpeed(0);
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
  }

  void TestPeriodic() {
  }

};

START_ROBOT_CLASS(Robot);
