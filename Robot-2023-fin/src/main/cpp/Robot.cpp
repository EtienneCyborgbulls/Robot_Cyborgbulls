// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

# include "Include.h"

void Robot::RobotInit() {
// This function is called when the robot program first starts up, 
// is used for initializing any necessary variables, objects, or hardware interfaces before the robot begins operating. 
// It is only called once during the runtime of the program.
  
  

   m_MotorRight.SetInverted(true);
  // This line sets the direction of the right-side motor to be inverted, meaning that positive motor commands will cause it to rotate in the opposite direction.
  // This can be useful for ensuring that the robot drives straight when both sides are given the same motor command.
  
   m_MotorRightFollow.SetInverted(true);
  // This line sets the direction of the right-side follower motor to be inverted, just like the previous line.
  
   m_MotorLeft.SetInverted(false);
  // This line sets the direction of the left-side motor to be normal, meaning that positive motor commands will cause it to rotate in the expected direction.
  
  m_MotorLeftFollow.SetInverted(false);
  // This line sets the direction of the left-side follower motor to be normal, just like the previous line.
  ;

  m_MotorRightFollow.Follow(m_MotorRight); // Pour que les 2 moteurs se suivent
  m_MotorLeftFollow.Follow(m_MotorLeft);   //

  m_MotorRight.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);       //
  m_MotorRightFollow.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR); //
  m_MotorLeft.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);        // Definition voltage maximum a donner
  m_MotorLeftFollow.ConfigVoltageCompSaturation(VOLTAGE_COMPENSATION_DRIVETRAIN_MOTOR);  // a chaque moteur
  
  m_ArmMotor.EnableVoltageCompensation(VOLTAGE_COMPENSATION_ARM_MOTOR);                  //
  m_IntakeRotor.EnableVoltageCompensation(VOLTAGE_COMPENSATION_ARM_MOTOR);               //
  
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);                        // Definition periode auto de defaut
  m_chooser.AddOption(kAutoNameCube, kAutoNameCube);                                     // Ajout de periode auto
  m_chooser.AddOption(kAutoNamePlotG, kAutoNamePlotG);                                     //
  m_chooser.AddOption(kAutoNamePlotD, kAutoNamePlotD);                                   //
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);                                // Ajout des periodes auto sur le Shuffleboard
 
    cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture(); 
    // Set the resolution
    camera.SetResolution(640/2, 480/2);
    camera.SetFPS(40);
    

   
   // set the position offset to half a rotation     
   m_encoder.SetPosition(-6);
}

void Robot::RobotPeriodic()
{
  frc2::CommandScheduler::GetInstance().Run(); 

  double rawValue = m_ultrasonic.GetValue();
  double voltValue = rawValue*5/4096;
  frc::SmartDashboard::PutNumber("ultrason_rawvalue",rawValue);
  frc::SmartDashboard::PutNumber("distance in meters",voltValue);

}

void Robot::setIntakeRotor(double percent, int amps) {

  m_IntakeRotor.Set(percent);
  m_IntakeRotor.SetSmartCurrentLimit(amps);

}

void Robot::setDriveMotors(double forward, double turn){
  
  double left = forward - turn;
  double right = forward + turn;
  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, right);
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput, left);
}

void Robot::setArmMotor(double percent, int amps){

  m_ArmMotor.Set(percent);
  m_ArmMotor.SetSmartCurrentLimit(amps);

}

void Robot::AutonomousInit() {
  
  m_autoSelected = m_chooser.GetSelected();
  fmt::print("Auto selected: {}\n", m_autoSelected);

 if (m_autoSelected == kAutoNameCube) {
    // Custom Auto goes here
    setDriveMotors(0.0, 0.0);
    m_timer.Reset();
    m_timer.Start();
  }

  else {
    // Default Auto goes here
    setDriveMotors(0.0, 0.0);
    m_timer.Reset();
    m_timer.Start();
  }
 
}


void Robot::AutonomousPeriodic() {

  double PositionEncoder = -m_encoder.GetPosition(); 
 
  if(m_autoSelected == kAutoNamePlotG){                                        // period auto plot 
    if(m_timer.Get()< 0.5_s){
      setArmMotor(-0.7, 40);
      setDriveMotors(0.0, 0.0);
    } else if(m_timer.Get()< 1.5_s) {
      setArmMotor(0.2*-0.7*sin(PositionEncoder*(1.0/152.0)*360.0), 40);
      setDriveMotors(0.0, 0.0);
    }else if(m_timer.Get()>1.7_s && m_timer.Get()<2.2_s){
      setArmMotor(0.7, 40);
    }else{
      setArmMotor(0.0, 40);
    }
    if(m_timer.Get() > 0.6_s && m_timer.Get() < 0.65_s){
      setIntakeRotor(0.7, 40);
    } else {
      setIntakeRotor(0.0, 40);
    }
    if(m_timer.Get() > 0.7_s && m_timer.Get() < 1.5_s){

      DoublePH.Set(frc::DoubleSolenoid::Value::kForward);

    } else {
      DoublePH.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if(m_timer.Get() > 1.5_s && m_timer.Get() < 4.5_s){
      setDriveMotors(-0.4, 0.0);


    } else if(m_timer.Get() > 5.1_s && m_timer.Get() < 5.6_s){
     setDriveMotors(0.0, 0.8);
    }else {
      setDriveMotors(0.0, 0.0);
    }

  }else if(m_autoSelected == kAutoNamePlotD){                                        // period auto plot 
    if(m_timer.Get()< 0.5_s){
      setArmMotor(-0.7, 40);
      setDriveMotors(0.0, 0.0);
    } else if(m_timer.Get()< 1.5_s) {
      setArmMotor(0.2*-0.7*sin(PositionEncoder*(1.0/152.0)*360.0), 40);
      setDriveMotors(0.0, 0.0);
    }else if(m_timer.Get()>1.7_s && m_timer.Get()<2.2_s){
      setArmMotor(0.7, 40);
    }else{
      setArmMotor(0.0, 40);
    }
    if(m_timer.Get() > 0.6_s && m_timer.Get() < 0.65_s){
      setIntakeRotor(0.7, 40);
    } else {
      setIntakeRotor(0.0, 40);
    }
    if(m_timer.Get() > 0.85_s && m_timer.Get() < 1.5_s){

      DoublePH.Set(frc::DoubleSolenoid::Value::kForward);

    } else {
      DoublePH.Set(frc::DoubleSolenoid::Value::kReverse);
    }
    if(m_timer.Get() > 1.5_s && m_timer.Get() < 4.8_s){
      setDriveMotors(-0.4, 0.1);


    } else if(m_timer.Get() > 5.1_s && m_timer.Get() < 5.6_s){
     setDriveMotors(0.0, -0.8);
    }else {
      setDriveMotors(0.0, 0.0);
    }
   }


}

void Robot::TeleopInit() {
  setDriveMotors(0.0, 0.0);
  setArmMotor(0.0, 40);
  setIntakeRotor(0.0, 40);
}

void Robot::TeleopPeriodic() {

double PositionEncoder = -m_encoder.GetPosition(); 

  double max = frc::SmartDashboard::GetNumber("Max Output", 0);
  double min = frc::SmartDashboard::GetNumber("Min Output", 0);



  

  frc::SmartDashboard::PutNumber("ProcessVariable",PositionEncoder*(1.0/152.0)*360.0);

  
  double vitesse; 
  vitesse = m_joystick.GetThrottle();                                       // Throttle
  if (m_joystick.GetThrottle() < 0) {
    vitesse = 0.4;
   }
    else if(m_joystick.GetThrottle()>0){
    vitesse = 0.8;
   }
  double y = -m_joystick.GetY();
  double z = m_joystick.GetZ();
  double forward = utils::Deadband(y);
  double turn = utils::Deadband(z);
  double v = forward * VMAX;
  double w = turn * WMAX * vitesse;                   


  double left_wheel = v + (w * HALF_TRACKWIDTH);
  double right_wheel = v - (w * HALF_TRACKWIDTH);

  double k;
  k = 1.0 / (NMAX(VMAX, NMAX(NABS(left_wheel), NABS(right_wheel))));
  left_wheel *= k;
  right_wheel *= k;

  m_MotorRight.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput,left_wheel); //
  m_MotorLeft.Set(ctre::phoenix::motorcontrol::TalonSRXControlMode::PercentOutput,  right_wheel); //



 
 double ArmPower;                                                            // Arm up
 if (m_joystick.GetRawButton(6)==true) {
   ArmPower = -0.7;
   if( PositionEncoder *(1.0/152.0)*360.0>105){
   ArmPower = 0.3*-0.7*sin(PositionEncoder*(1.0/152.0)*360.0);

  }

 }
 else if (m_joystick.GetRawButton(5)==true){                                // Arm down
   ArmPower = 0.4;
   if( PositionEncoder *(1.0/152.0)*360.0<20){
   ArmPower = 0.3*-0.7*sin(PositionEncoder*(1.0/152.0)*360.0);
  
 }
 }
 else {                                                                    // Arm compensed
  ArmPower = 0.3*-0.7*sin(PositionEncoder*(1.0/152.0)*360.0);
 }
 setArmMotor(ArmPower, 40);

 double IntakeRotorPower;

 if(m_joystick.GetRawButton(4)==true){                                     // Clamp holder up
 IntakeRotorPower = -0.9;


 
 } else if(m_joystick.GetRawButton(3)==true){                              // Clamp holder down
 IntakeRotorPower = 0.9;
 

 
 }else{                                                                    // Not compensed
  IntakeRotorPower = 0;
 }
 setIntakeRotor(IntakeRotorPower, 40);
 
 if(m_joystick.GetRawButton(1)==true){
 DoublePH.Set(frc::DoubleSolenoid::Value::kForward);
 } else if(m_joystick.GetRawButton(1)==false){

 DoublePH.Set(frc::DoubleSolenoid::Value::kReverse);

 }

 double pidOutput;
 if(m_joystick.GetRawButton(9)==true){                                    // ultrasonic
  double rawValue = m_ultrasonic.GetValue();
  m_pidController3.SetSetpoint(819);
  pidOutput = m_pidController3.Calculate(rawValue);
  
  if(pidOutput > 0.2){
      pidOutput = 0.4;
  }
  else if(pidOutput<-0.2 ){
    pidOutput = -0.4;
  }
  std::cout<< "output" << pidOutput <<std::endl;
  frc::SmartDashboard::PutNumber("output",pidOutput);
 setDriveMotors(pidOutput,0.0);

}else{
  pidOutput = 0.0;
  }



 }





#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif