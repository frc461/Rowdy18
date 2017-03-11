/*
 * Shooter.h
 *
 *  Created on: Mar 6, 2017
 *      Author: WBI
 */

#ifndef SRC_SHOOTER_H_
#define SRC_SHOOTER_H_

#include "PeriodicExecutable.h"

class BetterPIDController;
class RateCounter;
class SettablePIDOut;

enum ShootingMode {
  Automatic,
  AllOnManual,
  Manual
};

class Shooter: public PeriodicExecutable {
 public:
  Shooter(OperatorControls *controls);

  void Initialize();
  void Execute();
  void Log();
  
 private:
  Spark *leftShooter;
  Spark *leftTower;
  Spark *rightShooter;
  Spark *rightTower;

  Spark *conveyor;
  
  RateCounter *leftShooterEncoder;
  RateCounter *rightShooterEncoder;
  
  BetterPIDController *leftPID;
  BetterPIDController *rightPID;
  
  SettablePIDOut *leftOut;
  SettablePIDOut *rightOut;

  double shootingSpeed;
  bool usePIDForManualShooting;

  OperatorControls *controls;

  void AutomaticShooting();
  void AllOnManualShooting();
  void Shoot();
  void StopShooting();

  void RunConveyorManually();

  void RunTowersManually();

  void SetShootingSetpint(double setpoint);

  ShootingMode mode;
  double towerSpeed;
  double conveyorSpeed;
 
};

#endif /* SRC_SHOOTER_H_ */
