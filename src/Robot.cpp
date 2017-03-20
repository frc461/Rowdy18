#include <WPILib.h>
#include "Robot.h"
#include "XboxJoystickMap.h"
#include <Timer.h>
#include "SettablePIDOut.h"
#include "RateCounter.h"
#include "Logger.h"

#include "Shooter.h"
#include "DriveTrain.h"
#include "Intake.h"
#include "Climber.h"

#include "AutoCenterGear.h"
#include "AutoLeftGear.h"
#include "AutoRightGear.h"

#define USE_GYRO_DRIVE_CORRECTION

//Gyro, left is positive, right is negative

class Robot: public IterativeRobot {
  Joystick op;
  Timer timer;
  AnalogInput currentSensor;
  int state = 0;
  double initialAngle = -1;
  PowerDistributionPanel *pdp = new PowerDistributionPanel();
  double driveAngle = -1;
  int gRPulseInit;
  double gRAngleInit;

  DriverControls *driverControls = DriverControls::SharedDriverControls();
  DriveTrain *driveTrain = new DriveTrain(driverControls);

  OperatorControls *operatorControls = OperatorControls::SharedOperatorControls();
  Shooter *shooter = new Shooter(operatorControls);
  Intake *intake = new Intake(operatorControls);
  Climber *climber = new Climber(operatorControls);
  AutoBase *currentAuto;

  AutoCenterGear *centerGearAuto = new AutoCenterGear(driveTrain);
  AutoLeftGear *leftGearAuto = new AutoLeftGear(driveTrain);
  AutoRightGear *rightGearAuto = new AutoRightGear(driveTrain);

  SendableChooser<AutoBase*> *autoChooser = new SendableChooser<AutoBase*>();

public:
  Robot() :
    op(1),
    timer(),
    currentSensor(3)
{
    SmartDashboard::init();
    //b = DriverStationLCD::GetInstance();
  }

private:
  void RobotInit() {
            
            driveAngle = -1;

            autoChooser->AddDefault("Center gear", centerGearAuto);
            autoChooser->AddObject("Right gear", rightGearAuto);
            autoChooser->AddObject("Left gear", leftGearAuto);
            SmartDashboard::PutData("Auto Chooser", autoChooser);
  }

  /*
  void DriveStraight(double speed) {
	  double correction;
	  double currentAngle = 0;
#ifdef USE_GYRO_DRIVE_CORRECTION
	  if (driveAngle == -1.0) {
		  driveAngle = fmod(gyro.GetAngle(), 360);
	  }

	  if (speed == 0) {
		  driveAngle = -1;
	  }

	  currentAngle = fmod(gyro.GetAngle(), 360);
	  if (currentAngle != 0.0) {
		  if (currentAngle > 180) {
			  currentAngle -= 360;
		  }

		  double error = currentAngle - driveAngle;
		  correction = (error / 180) * 2.8;
	  } else {
		  correction = 0;
	  }
#else
	  correction = 0;
#endif

	  driveTrain.TankDrive(speed - correction, speed + correction);

	  logger->Log(logDriveTrain, "Current angle %lf, target angle: %lf, correction: %lf\n", currentAngle, driveAngle, correction);
  }

  bool TurnAngle(double targetAngle){
	  double currentAngle = fmod(gyro.GetAngle(), 360);

	  if(fabs(currentAngle - targetAngle) > 3) { //Magic deadzone
		  if(currentAngle > targetAngle){
			  driveTrain.TankDrive(-0.5, .05);
		  }else if (currentAngle < targetAngle) {
			  driveTrain.TankDrive(0.5, -0.5);
		  }
		  return false;
	  } else {
		  return true;
	  }
  }

  void DisabledInit() {
	  logger->CloseLog();
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
//      AutomaticShooting();
      if (timer.Get() > 5) {
        timer.Reset();
        state++;
      }
    } else {
//      StopShooting();
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
	  shifter.Set(SHIFTER_LOW);
	  switch (state) {
	  case CG_Init:
		  timer.Reset();
		  timer.Start();
		  leftDriveEncoder.Reset();
		  rightDriveEncoder.Reset();
		  driveAngle = -1;
		  state++;
		  break;

	  case CG_Drive:
		  DriveStraight(DRIVE_BACKWARD_SPEED(.6));

		  if (timer.Get() > 1) {
			  intake.Set(INTAKE_DOWN);
		  } else {
			  intake.Set(INTAKE_UP);
		  }

		  if (leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(65)) {
			  state++;
		  }
		  break;

	  case CG_FINISH:
		  DriveStraight(DRIVE_BACKWARD_SPEED(0));
	  }
  }

  void autoLeftGearReload() {
	  logger->Log(logAuton, "Left drive: %d, right drive: %d, gyro: %lf\n", leftDriveEncoder.Get(), rightDriveEncoder.Get(), gyro.GetAngle());
	  logger->Log(logAuton, "Auton state autoLeftGearReload is in state %d\n", state);
	if (state == lGR_Init) {
		timer.Reset();
		timer.Start();
		logger->Log(logAuton, "Timer reset\n");
		state++;
	}
	else if (state == lGR_LowGear) {
      shifter.Set(SHIFTER_LOW);
	  logger->Log(logAuton, "Shifted to low gear\n");
      if (timer.Get() > 1) {
    	  timer.Reset();
    	  timer.Start();
  		logger->Log(logAuton, "1 second passed, timer reset\n");
        state++;
      }
    }
    else if (state == lGR_BackUp0Init) {
    	gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set at %d\n", gRPulseInit);
    	state++;
    }
    else if (state == lGR_BackUp0) {
      DriveStraight(DRIVE_BACKWARD_SPEED(.5));
	  logger->Log(logAuton, "Driving backwards at 0.5 power\n");
      if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(12) + gRPulseInit) {
          DriveStraight(0);
          driveAngle = -1;
  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
    	  state++;
    	  timer.Reset();
    	  timer.Start();
  		  logger->Log(logAuton, "Timer reset\n");
      }
    }
    else if (state == lGR_LowerIntake) {
        intake.Set(INTAKE_DOWN);
		logger->Log(logAuton, "Intake put down\n");
        if (timer.Get() > 1) {
          timer.Reset();
          logger->Log(logAuton, "1 second passed, timer reset\n");
          state++;
        }
      }
    else if (state == lGR_BackUp1Init) {
      	gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set\n");
      	state++;
    }
    else if (state == lGR_BackUp1) {
    	  DriveStraight(DRIVE_BACKWARD_SPEED(.5));
    	  logger->Log(logAuton, "Driving backwards at 0.5 power\n");
		  if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(50) + gRPulseInit) {
			  DriveStraight(0);
	  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			  state++;
		  }
    }
    else if (state == lGR_RotateRight0Init) {
            	gRAngleInit = fmod(gyro.GetAngle(), 360);
        		logger->Log(logAuton, "Init angle set\n");
            	state++;
    }
    else if (state == lGR_RotateRight0) {
    	if (TurnAngle(fmod((gRAngleInit + 60), 360))) {
    	//Intentionally Empty "If Statement"
    		logger->Log(logAuton, "Turning 60 degrees\n");
    	} else {
    		DriveStraight(0);
	  		logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
    		state++;
    	}
	}
	else if (state == lGR_BackUp2Init){
		gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set\n");
		state++;
	}
	else if (state == lGR_BackUp2) {
		DriveStraight(DRIVE_BACKWARD_SPEED(.5));
  	    logger->Log(logAuton, "Driving backwards at 0.5 power\n");
		if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(17) + gRPulseInit){
			DriveStraight(0);
	  		logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			state++;
		}
	}
  }

  void autoLGRHopperRed() {
	  if (state <= lGRHR_LeftGearReload){
		  autoLeftGearReload();
	  } else {
	  switch (state) {
	  case lGRHR_Init:
		  timer.Reset();
		  timer.Start();
		  leftDriveEncoder.Reset();
		  rightDriveEncoder.Reset();
		  break;

	  case lGRHR_DriveForward0:
		  if (timer.Get() > 2) {
			  DriveStraight(DRIVE_FORWARD_SPEED(.5));
			  logger->Log(logAuton, "Driving forward at 0.5 power\n");

			  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(46.625)) {
				  DriveStraight(0);
		  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
				  state++;
			  }
		  }
		  break;

	  case lGRHR_RotateRight0Init:
		  gRAngleInit = fmod(gyro.GetAngle(), 360);
		  logger->Log(logAuton, "Init angle set\n");
		  state++;
		  break;

	  case lGRHR_RotateRight0:
		  if (TurnAngle(fmod((gRAngleInit + 30), 360))) {
			//Intentionally Empty "If Statement"
				logger->Log(logAuton, "Turning 30 degrees\n");
	      } else {
				DriveStraight(0);
				logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
				state++;
		  }
		  break;
	  case lGRHR_DriveForward1:
		  DriveStraight(DRIVE_FORWARD_SPEED(.5));
		  logger->Log(logAuton, "Driving forward at 0.5 power\n");

		  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(42.5)) {
			  DriveStraight(0);
			  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			  state++;
		  }
		  break;
	  }
	}
  }

  void autoLGRHopperBlue() {
	  if (state <= lGRHB_LeftGearReload){
	  		  autoLeftGearReload();
	  	  } else {
	  	  switch (state) {
	  	  case lGRHB_Init:
	  		  timer.Reset();
	  		  timer.Start();
	  		  leftDriveEncoder.Reset();
	  		  rightDriveEncoder.Reset();
	  		  break;

	  	  case lGRHB_DriveForward0:
	  		  if (timer.Get() > 2) {
	  			  DriveStraight(DRIVE_FORWARD_SPEED(.5));
	  			  logger->Log(logAuton, "Driving forward at 0.5 power\n");

	  			  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(52)) {
	  				  DriveStraight(0);
	  		  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
	  				  state++;
	  			  }
	  		  }
	  		  break;

	  	  case lGRHB_RotateRight0Init:
	  		  gRAngleInit = fmod(gyro.GetAngle(), 360);
	  		  logger->Log(logAuton, "Init angle set\n");
	  		  state++;
	  		  break;

	  	  case lGRHB_RotateRight0:
	  		  if (TurnAngle(fmod((gRAngleInit + 90), 360))) {
	  			//Intentionally Empty "If Statement"
	  				logger->Log(logAuton, "Turning 90 degrees\n");
	  	      } else {
	  				DriveStraight(0);
	  				logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
	  				state++;
	  		  }
	  		  break;

	  	  case lGRHB_DriveForward1:
	  		  DriveStraight(DRIVE_FORWARD_SPEED(.5));
	  		  logger->Log(logAuton, "Driving forward at 0.5 power\n");

	  		  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(91.75)) {
	  			  DriveStraight(0);
	  			  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
	  			  state++;
	  		  }
	  		  break;

	  	case lGRHB_RotateLeft0Init:
			  gRAngleInit = fmod(gyro.GetAngle(), 360);
			  logger->Log(logAuton, "Init angle set\n");
			  state++;
			  break;

	    case lGRHB_RotateLeft0:
			  if (TurnAngle(fmod((gRAngleInit - 60), 360))) {
				//Intentionally Empty "If Statement"
					logger->Log(logAuton, "Turning -60 degrees\n");
			  } else {
					DriveStraight(0);
					logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
					state++;
			  }
			  break;

	  	case lGRHB_DriveForward2:
			  DriveStraight(DRIVE_FORWARD_SPEED(.5));
			  logger->Log(logAuton, "Driving forward at 0.5 power\n");

			  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(13.5)) {
				  DriveStraight(0);
				  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
				  state++;
			  }
			  break;
	  	  }
	  }
  }

  void autoRightGearReload() {
	  logger->Log(logAuton, "Left drive: %d, right drive: %d, gyro: %lf\n", leftDriveEncoder.Get(), rightDriveEncoder.Get(), gyro.GetAngle());
	  logger->Log(logAuton, "Auton state autoLeftGearReload is in state %d\n", state);
	if (state == rGR_Init) {
		timer.Reset();
		timer.Start();
		logger->Log(logAuton, "Timer reset\n");
		state++;
	}
	else if (state == rGR_LowGear) {
      shifter.Set(SHIFTER_LOW);
	  logger->Log(logAuton, "Shifted to low gear\n");
      if (timer.Get() > 1) {
    	  timer.Reset();
    	  timer.Start();
  		logger->Log(logAuton, "1 second passed, timer reset\n");
        state++;
      }
    }
    else if (state == rGR_BackUp0Init) {
    	gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set at %d\n", gRPulseInit);
    	state++;
    }
    else if (state == rGR_BackUp0) {
      DriveStraight(DRIVE_BACKWARD_SPEED(.5));
	  logger->Log(logAuton, "Driving backwards at 0.5 power\n");
      if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(12) + gRPulseInit) {
          DriveStraight(0);
          driveAngle = -1;
  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
    	  state++;
    	  timer.Reset();
    	  timer.Start();
  		  logger->Log(logAuton, "Timer reset\n");
      }
    }
    else if (state == rGR_LowerIntake) {
        intake.Set(INTAKE_DOWN);
		logger->Log(logAuton, "Intake put down\n");
        if (timer.Get() > 1) {
          timer.Reset();
          logger->Log(logAuton, "1 second passed, timer reset\n");
          state++;
        }
      }
    else if (state == rGR_BackUp1Init) {
      	gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set\n");
      	state++;
    }
    else if (state == rGR_BackUp1) {
    	  DriveStraight(DRIVE_BACKWARD_SPEED(.5));
    	  logger->Log(logAuton, "Driving backwards at 0.5 power\n");
		  if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(50) + gRPulseInit) {
			  DriveStraight(0);
	  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			  state++;
		  }
    }
    else if (state == rGR_RotateRight0Init) {
            	gRAngleInit = fmod(gyro.GetAngle(), 360);
        		logger->Log(logAuton, "Init angle set\n");
            	state++;
    }
    else if (state == rGR_RotateRight0) {
    	if (TurnAngle(fmod((gRAngleInit - 60), 360))) {
    	//Intentionally Empty "If Statement"
    		logger->Log(logAuton, "Turning -60 degrees\n");
    	} else {
    		DriveStraight(0);
	  		logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
    		state++;
    	}
	}
	else if (state == rGR_BackUp2Init){
		gRPulseInit = leftDriveEncoder.Get();
		logger->Log(logAuton, "Init pulse set\n");
		state++;
	}
	else if (state == rGR_BackUp2) {
		DriveStraight(DRIVE_BACKWARD_SPEED(.5));
  	    logger->Log(logAuton, "Driving backwards at 0.5 power\n");
		if(leftDriveEncoder.Get() < -DRIVE_DISTANCE_INCHES(17) + gRPulseInit){
			DriveStraight(0);
	  		logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			state++;
		}
	}
  }

  void autoRGRHopperRed() {
  	  if (state <= rGRHR_RightGearReload){
  		  autoRightGearReload();
  	  } else {
  	  switch (state) {
  	  case rGRHR_Init:
  		  timer.Reset();
  		  timer.Start();
  		  leftDriveEncoder.Reset();
  		  rightDriveEncoder.Reset();
  		  break;

  	  case rGRHR_DriveForward0:
  		  if (timer.Get() > 2) {
  			  DriveStraight(DRIVE_FORWARD_SPEED(.5));
  			  logger->Log(logAuton, "Driving forward at 0.5 power\n");

  			  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(32)) {
  				  DriveStraight(0);
  		  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
  				  state++;
  			  }
  		  }
  		  break;

  	  case rGRHR_RotateLeft0Init:
  		  gRAngleInit = fmod(gyro.GetAngle(), 360);
  		  logger->Log(logAuton, "Init angle set\n");
  		  state++;
  		  break;

  	  case rGRHR_RotateLeft0:
  		  if (TurnAngle(fmod((gRAngleInit - 90), 360))) {
  			//Intentionally Empty "If Statement"
  				logger->Log(logAuton, "Turning -90 degrees\n");
  	      } else {
  				DriveStraight(0);
  				logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
  				state++;
  		  }
  		  break;

  	  case rGRHR_DriveForward1:
  		  DriveStraight(DRIVE_FORWARD_SPEED(.5));
  		  logger->Log(logAuton, "Driving forward at 0.5 power\n");

  		  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(91.75)) {
  			  DriveStraight(0);
  			  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
  			  state++;
  		  }
  		  break;

  	  case rGRHB_RotateRight0Init:
  		  gRAngleInit = fmod(gyro.GetAngle(), 360);
  		  logger->Log(logAuton, "Init angle set\n");
  		  state++;
  		  break;

  	  case rGRHB_RotateRight0:
  		  if (TurnAngle(fmod((gRAngleInit + 60), 360))) {
				//Intentionally Empty "If Statement"
					logger->Log(logAuton, "Turning 60 degrees\n");
		  } else {
				DriveStraight(0);
				logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
				state++;
		  }
		  break;

		case rGRHR_DriveForward2:
			DriveStraight(DRIVE_FORWARD_SPEED(.5));
			logger->Log(logAuton, "Driving forward at 0.5 power\n");

			if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(13.5)) {
			  DriveStraight(0);
			  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
			  state++;
			}
		break;
	}
  }
}

  void autoRGRHopperBlue() {
 	  if (state <= rGRHB_RightGearReload){
 	  		  autoLeftGearReload();
 	  	  } else {
 	  	  switch (state) {
 	  	  case lGRHB_Init:
 	  		  timer.Reset();
 	  		  timer.Start();
 	  		  leftDriveEncoder.Reset();
 	  		  rightDriveEncoder.Reset();
 	  		  break;

 	  	  case rGRHB_DriveForward0:
 	  		  if (timer.Get() > 2) {
 	  			  DriveStraight(DRIVE_FORWARD_SPEED(.5));
 	  			  logger->Log(logAuton, "Driving forward at 0.5 power\n");

 	  			  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(46.625)) {
 	  				  DriveStraight(0);
 	  		  		  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
 	  				  state++;
 	  			  }
 	  		  }
 	  		  break;

 	  	  case rGRHB_RotateLeft0Init:
 	  		  gRAngleInit = fmod(gyro.GetAngle(), 360);
 	  		  logger->Log(logAuton, "Init angle set\n");
 	  		  state++;
 	  		  break;

 	  	  case rGRHB_RotateLeft0:
 	  		  if (TurnAngle(fmod((gRAngleInit - 30), 360))) {
 	  			//Intentionally Empty "If Statement"
 	  				logger->Log(logAuton, "Turning -30 degrees\n");
 	  	      } else {
 	  				DriveStraight(0);
 	  				logger->Log(logAuton, "Angle change amount reached, stopping movement\n");
 	  				state++;
 	  		  }
 	  		  break;

 	  	  case rGRHB_DriveForward1:
 	  		  DriveStraight(DRIVE_FORWARD_SPEED(.5));
 	  		  logger->Log(logAuton, "Driving forward at 0.5 power\n");

 	  		  if (leftDriveEncoder.Get() > DRIVE_DISTANCE_INCHES(42.5)) {
 	  			  DriveStraight(0);
 	  			  logger->Log(logAuton, "Pulse amount reached, stopping movement\n");
 	  			  state++;
 	  		  }
 	  		  break;
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
//      AutomaticShooting();
      if (timer.Get() > 5) {
        timer.Reset();
        state++;
      }
    } else {
//      StopShooting();
    }
  }
  */

  void AutonomousInit() {
    currentAuto = autoChooser->GetSelected();
    state = 0;
    initialAngle = -1;

    timer.Start();
    Logger::OpenNewLog("_auton");

    currentAuto->Initialize();

    Initialize();
  }

  void AutonomousPeriodic() {
    Logger::LogRunTime();
    currentAuto->Execute();
//	  logger->LogRunTime();
//	  logger->Log(logAuton, "Auton is in state %d\n", state);
//    switch (mode) {
//    case rightGearHighGoal:
//      autoRightGearHighGoal();
//      break;
//    case rightGearHighGoalReload:
//      autoRightGearHighGoalReload();
//      break;
//    case centerGear:
//      autoCenterGear();
//      break;
//    case leftGearReload:
//      autoLeftGearReload();
//      break;
//    case lGRHopperRed:
//	  autoLGRHopperRed();
//	  break;
//    case lGRHopperBlue:
//      autoLGRHopperBlue();
//      break;
//    case rightGearReload:
//      autoRightGearReload();
//      break;
//    case rGRHopperRed:
//      autoRGRHopperRed();
//      break;
//    case rGRHopperBlue:
//      autoRGRHopperBlue();
//      break;
//    case highGoalReload:
//      autoHighGoalReload();
//      break;
//    default:
//      std::cout << "you screwed up" << std::endl;
//      logger->Log(logAuton, "Ran invalid auton\n");
//    }
  }

  void Initialize() {
    driveTrain->Initialize();
    climber->Initialize();
    shooter->Initialize();
    intake->Initialize();

  }

  void TeleopInit() {
    Logger::OpenNewLog("_teleop");
    Initialize();
  }

  void Monitor() {
    driveTrain->Log();
    climber->Log();
    shooter->Log();
    intake->Log();
  }

  void TeleopPeriodic() {
    Logger::LogRunTime();

    driveTrain->Execute();

    climber->Execute();
    shooter->Execute();
    intake->Execute();

    Monitor();
  }

  void TestPeriodic() {
  }

};

START_ROBOT_CLASS(Robot);
