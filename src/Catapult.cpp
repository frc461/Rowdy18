/*
 * Catapult.cpp
 *
 *  Created on: Apr 19, 2017
 *      Author: WBI
 */

#include <Catapult.h>


Catapult::Catapult(OperatorControls *controls) {
  // TODO Auto-generated constructor stub

  solenoid1 = new DoubleSolenoid(5, 6);
  solenoid2 = new DoubleSolenoid(3, 4);

  this->controls = controls;
}

void Catapult::Initialize(){
  solenoid1->Set(DoubleSolenoid::kForward);
  solenoid2->Set(DoubleSolenoid::kForward);
}

void Catapult::Execute(){
  if(controls->GetCatapultDirection() == Direction::kForward  && controls->GetClimberDirection() == Direction::kOff){
    solenoid1->Set(DoubleSolenoid::kReverse);
    solenoid2->Set(DoubleSolenoid::kReverse);
  }else{
    solenoid1->Set(DoubleSolenoid::kForward);
    solenoid2->Set(DoubleSolenoid::kForward);
  }
}

void Catapult::Log(){

}


