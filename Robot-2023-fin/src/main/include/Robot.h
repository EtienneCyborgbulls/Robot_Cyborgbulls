// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include <cstdio>
#include <span>
#include <sstream>
#include <string>
#include <thread>
#include <frc/TimedRobot.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/NetworkTableInstance.h>

#include <units/angle.h>
#include <units/length.h>
#include <cameraserver/CameraServer.h>
#include <fmt/format.h>
#include <frc/Joystick.h>
#include "Constants.h"
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/motorcontrol/Talon.h>
#include "rev/CANSparkMax.h"
#include <frc/XboxController.h>
#include <frc2/command/PIDCommand.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <iostream>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <math.h>

#include <frc/Encoder.h>
#include <units/pressure.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/PneumaticsControlModule.h>

#include <frc/Ultrasonic.h>
#include <frc/AnalogInput.h>
#include <frc/filter/MedianFilter.h>

class Robot : public frc::TimedRobot 

{
public:
 void setDriveMotors(double forward, double turn);
  void RobotInit() override;
  void RobotPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void setArmMotor(double percent, int amps);
  void setIntakeRotor(double percent, int amps);
  

private:
 
 frc::Joystick m_joystick{0};
 frc::XboxController m_XboxController{1};
 frc::Timer m_timer;
 frc::SendableChooser<std::string> m_chooser;
 frc::AnalogInput m_ultrasonic{0};


 const std::string kAutoNameDefault = "Default";
 const std::string kAutoNameCube = "Cube";
 const std::string kAutoNamePlotG = "Plot Gauche";
 const std::string kAutoNamePlotD = "Plot Droit";
 std::string m_autoSelected;

 rev::CANSparkMax m_ArmMotor{CAN_ID_ARM_MOTOR, rev::CANSparkMax::MotorType::kBrushless};
 rev::CANSparkMax m_IntakeRotor{CAN_ID_INTAKE_ROTOR, rev::CANSparkMax::MotorType::kBrushed};
 ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRight{CAN_ID_DRIVETRAIN_MOTOR_RIGHT};
 ctre::phoenix::motorcontrol::can::TalonSRX m_MotorRightFollow{CAN_ID_DRIVETRAIN_MOTOR_RIGHT_FOLLOW};
 ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeft{CAN_ID_DRIVETRAIN_MOTOR_LEFT};
 ctre::phoenix::motorcontrol::can::TalonSRX m_MotorLeftFollow{CAN_ID_DRIVETRAIN_MOTOR_LEFT_FOLLOW};
 
 std::function<double()> m_Forward;
 std::function<double()> m_Turn;
 std::function<double()> m_Slide;

 frc::Compressor phCompressor{10, frc::PneumaticsModuleType::REVPH};
 frc::DoubleSolenoid DoublePH{10, frc::PneumaticsModuleType::REVPH, 1, 2};	
 bool enabled = phCompressor.Enabled();

 frc2::PIDController m_pidController3{-0.001,0.0,0.0};

 rev::SparkMaxRelativeEncoder m_encoder = m_ArmMotor.GetEncoder();


};