#ifndef ROBOT
#define ROBOT

#pragma once

#include <string>

#include <frc/AnalogInput.h>
#include <frc/DigitalInput.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
// #include "ctre/Phoenix.h"
#include "rev/CANSparkMax.h"
#include <frc/motorcontrol/Spark.h>
#include <frc/DutyCycleEncoder.h>
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include "frc/motorcontrol/PWMMotorController.h"
#include <frc/Solenoid.h>
// #include <networktables/NetworkTable.h>
#include <photon/PhotonCamera.h>
// #include <photonlib/PhotonUtils.h>
// #include <cstdio>
// #include <cameraserver/CameraServer.h>

#include "Const.hpp"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TestPeriodic() override;
  void RobotMotorCommands();

  // Analog Inputs Test
  // Practice Bot Wheel Angle Encoders
  #ifdef PracticeBot
  frc::AnalogInput a_encoderFrontLeftSteer{2};
  frc::AnalogInput a_encoderFrontRightSteer{1};
  frc::AnalogInput a_encoderRearLeftSteer{3};
  frc::AnalogInput a_encoderRearRightSteer{0};
  #endif
 
  
  //DIO - Inputs / Outputs
  #ifdef CompBot
  // WPI_CANCoder          m_encoderWheelAngleCAN_FL     {KeEnc_i_WheelAngleFL, "rio"};
  // WPI_CANCoder          m_encoderWheelAngleCAN_FR     {KeEnc_i_WheelAngleFR, "rio"};
  // WPI_CANCoder          m_encoderWheelAngleCAN_RL     {KeEnc_i_WheelAngleRL, "rio"};
  // WPI_CANCoder          m_encoderWheelAngleCAN_RR     {KeEnc_i_WheelAngleRR, "rio"};
  
  // frc::DigitalInput     di_TurrentLimitSwitch {C_TurretSensorID};
  // frc::DigitalOutput    do_CameraLightControl {C_CameraLightControl_ID};
  #endif

  // PDP - Power Distribution Panel - CAN
  frc::PowerDistribution                     PDP                   {C_PDP_ID,               frc::PowerDistribution::ModuleType::kRev};

  // CAN Motor Controllers
  rev::CANSparkMax                           m_Motor1 {1,  rev::CANSparkMax::MotorType::kBrushless};
  rev::CANSparkMax                           m_Motor2 {2,  rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_frontRightSteerMotor{frontRightSteerDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_frontRightDriveMotor{frontRightDriveDeviceID, rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_rearLeftSteerMotor  {rearLeftSteerDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_rearLeftDriveMotor  {rearLeftDriveDeviceID,   rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_rearRightSteerMotor {rearRightSteerDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  // rev::CANSparkMax                           m_rearRightDriveMotor {rearRightDriveDeviceID,  rev::CANSparkMax::MotorType::kBrushless};
  //frc::Spark spark1{0};
  //frc::Spark spark2{1};
  //frc::Spark spark3{2};
  //frc::Spark spark4{3};

  rev::SparkMaxPIDController                 m_Motor1_PID   = m_Motor1.GetPIDController();
  rev::SparkMaxPIDController                 m_Motor2_PID   = m_Motor2.GetPIDController();

  // // CAN Encoders
  rev::SparkRelativeEncoder               m_Motor1Encoder  = m_Motor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_Motor2Encoder  = m_Motor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);


  // Driver Inputs
  frc::Joystick c_joyStick{0};
#ifdef CompBot
  frc::Joystick c_joyStick2{1};
#endif

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
};
#endif
