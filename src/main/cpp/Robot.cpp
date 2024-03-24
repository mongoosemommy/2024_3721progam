#include "subsystems/Drive.hpp"
#include "Robot.h"
#include <hal/HAL.h>
#include <rev/REVCommon.h>
#include <fmt/core.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/TimedRobot.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include "rev/CANSparkMax.h"
#include <frc/Joystick.h>
#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc2/command/button/CommandJoystick.h>
#include <frc2/command/Command.h>
#include <frc2/command/button/trigger.h>
#include <frc/GenericHID.h>
#include "math.h"

double currentTimeStamp = 0, lastTimestamp = 0, dt = 0;
double matchTime = 0;
double rotations;
double::Robot::Sign(double num){
  if(num < 0){
    return -1;
  }
  if(num > 0){
    return 1;
  }
}
void Robot::RobotInit() {  
//  this->curr_arm_target = manip->getInstance().kARM_START_POS;
  
    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */

  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption("Basic", "Basic");  
  m_chooser.AddOption("Multi Note", "MultiNote");
  m_chooser.AddOption("Send It", "SendIt");
  
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
  frc::SmartDashboard::PutNumber("Test", 4);

  xbox = new frc::XboxController(0);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {
  matchTime = (double)frc::Timer::GetMatchTime();
  currentTimeStamp = (double)frc::Timer::GetFPGATimestamp();
  dt = currentTimeStamp - lastTimestamp;
  

  lastTimestamp = currentTimeStamp;
}

bool testinit;

void Robot::AutonomousInit() {
  testinit = true;
  
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

  this->basic = new autonomous::Basic();
  this->multinote = new autonomous::MultiNote();
  this->sendit = new autonomous::SendIt();
}


void Robot::AutonomousPeriodic() {
  // double p = frc::SmartDashboard::GetNumber("P Gain", 0);
  // double i = frc::SmartDashboard::GetNumber("I Gain", 0);
  // double d = frc::SmartDashboard::GetNumber("D Gain", 0);
  // double iz = frc::SmartDashboard::GetNumber("I Zone", 0);
  // double ff = frc::SmartDashboard::GetNumber("Feed Forward", 0);
  // double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  // double min = frc::SmartDashboard::GetNumber("Min Output", 0);
  // double rotations = frc::SmartDashboard::GetNumber("Set Rotations", 0);
  
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {  
  /* Drive */
  double deadband = 0.1;
  double power = -xbox->GetRawAxis(1);     // 1 -- Left Y Axis
  double steering = xbox->GetRawAxis(4);  // 4 -- Rght X Axis
  
  double y;
  double x;
  //Check this 
  if (abs(power) < deadband){
    y = 0;
  }
  else{
    y = (1 / (1 - deadband)) * (power + (-Sign(power) * deadband));
  }

  if (abs(steering) < deadband){
    x = 0;
  }
  else{
    x = (1 / (1 - deadband)) * (steering + (-Sign(steering) * deadband));
  }
  //Change back if no work .move(power, steering);
  drive->getInstance().move(y, x);
  
  /* Intake */
  if (m_operatorPannel.GetRawButton(1)){
    // If pressing intake button, and the NOTE is not in the intake
    manip->getInstance().intake(-0.5);
    //if (xbox->GetRightTriggerAxis() < 0.5) {
      //this->curr_arm_target = manip->getInstance().kARM_FLOOR_POS;
   // }
  } else if (m_operatorPannel.GetRawButton(2)) {
    // Outtake
    manip->getInstance().intake(0.7);
    manip->getInstance().shoot(-0.25);
  } else {
    // Do nothing
    //manip->getInstance().intake(0.0);
    manip->getInstance().shoot(0.0);
    manip->getInstance().intake(0.0);
  }

  /*if (xbox->GetRightBumper() && !manip->getInstance().get_note_sensor()) {
    // If pressing intake and NOTE is in the intake
    xbox->SetRumble(frc::GenericHID::kBothRumble, 1.0);
  } else {
    xbox->SetRumble(frc::GenericHID::kBothRumble, 0.0);
  }*/
 
  if (m_operatorPannel.GetRawButtonReleased(1)&& m_operatorPannel.GetRawButtonReleased(2)) {
    // No longer intaking; raise intake to avoid  
    //this->curr_arm_target = manip->getInstance().kARM_FENDER_POS;

  //L1_pidController.SetReference(rotations, rev::CANSparkMax::ControlType::kPosition);
  
  //frc::SmartDashboard::PutNumber("SetPoint", rotations);
  //frc::SmartDashboard::PutNumber("ProcessVariable", R1_encoder.GetPosition());
  }

  /* Shooter */
  if (m_operatorPannel.GetRawButton(4)) {
    if (manip->getInstance().get_arm_enc() < manip->getInstance().kARM_START_POS) {
      // If arm turned back farther than starting config
      manip->getInstance().shoot(0.25);
    } else {
      // High goal shooting
      // Set shot angle
      // std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
      // double ty = table->GetNumber("ty", 0.0);
      // double shot_angle = -0.00008 * pow(ty,2) + .00252*ty + .4992;
      // this->curr_arm_target = shot_  angle;
    }

    // vision aiming
    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
    double tx = table->GetNumber("tx", 0.0);
    double Kp = 0.05;
    drive->getInstance().move(power, Kp * tx);
    frc::SmartDashboard::PutNumber("tx", tx);
  }

  if (m_operatorPannel.GetRawButton(3)) {
    manip->getInstance().intake(-1.0);
    manip->getInstance().shoot(-0.5);
    // if (manip->getInstance().get_arm_enc() < manip->getInstance().kARM_START_POS) {
    //   // If arm turned back farther than starting config, score AMP
    //   manip->getInstance().intake(1.0);
    //   manip->getInstance().shoot(0.5);
    // } else {
    //   // High goal shooting
    //   // Adjustable by driver. 50% press => 0% power, 100% press => 100% power
    //   manip->getInstance().shoot(0.75);
    // }

    //if (m_operatorPannel.GetRawButton(1)) {
      // Run intake despite NOTE being in intake
    //  manip->getInstance().intake(1.0);
   // }
   // } else {
    // Do nothing
    // manip->getInstance().intake(0.0);
    //manip->getInstance().shoot(0.0);
  }

  /* Arm manual control */
  if (m_operatorPannel.GetRawButton(11)) {
    // Amp scoring config
    this->curr_arm_target = this->manip->getInstance().kARM_AMP_POS;
  }

  if (m_operatorPannel.Button(12).Get()) {
    manip->getInstance().move_arm(0.5);  // Up
    this->curr_arm_target = this->manip->getInstance().get_arm_enc();
  } else if (m_operatorPannel.GetRawButton(6)) {
    manip->getInstance().move_arm(-0.5);  // Down
    this->curr_arm_target = this->manip->getInstance().get_arm_enc();
  } else {
    // Move arm to preset target, or current position if last command was manual control.
    //manip->getInstance().arm_to_pos(curr_arm_target);
  }
  frc::SmartDashboard::PutNumber("Arm", manip->getInstance().get_arm_enc());
  
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::TestInit() {}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif