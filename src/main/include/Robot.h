#ifndef ROBOT
#define ROBOT

#pragma once

#include <string>

#include <ctre/Phoenix.h>
#include <ctre/phoenix/motorcontrol/can/TalonFX.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/signals/SpnEnums.hpp>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/Serializable.hpp>
#include <ctre/phoenix6/controls/DutyCycleOut.hpp>
#include <ctre/phoenix6/controls/PositionDutyCycle.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>
#include <ctre/phoenix6/core/CoreCANcoder.hpp>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>

#include <frc/DigitalInput.h>
#include <frc/drive/DifferentialDrive.h>
#include <frc/DigitalOutput.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc/TimedRobot.h>
#include <frc/PowerDistribution.h>
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

using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::sensors;

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
  static constexpr char const *kCANBus{"canivore"};
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
  ctre::phoenix6::hardware::TalonFX m_krakentest{0, kCANBus};

  rev::SparkMaxPIDController                 m_Motor1_PID   = m_Motor1.GetPIDController();
  rev::SparkMaxPIDController                 m_Motor2_PID   = m_Motor2.GetPIDController();
  // ctre::phoenix6::hardware::TalonFX          m_krakentestPID   =m_krakentest.Get();
  
 // CAN Encoders
  rev::SparkRelativeEncoder               m_Motor1Encoder  = m_Motor1.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);
  rev::SparkRelativeEncoder               m_Motor2Encoder  = m_Motor2.GetEncoder(rev::SparkRelativeEncoder::Type::kHallSensor,42);

  // Driver Input
  frc::Joystick c_joyStick{0};
#ifdef CompBot
  frc::Joystick c_joyStick2{1};
#endif

 private:
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  ctre::phoenix6::hardware::CANcoder cancoder{1, "rio"};
  units::time::second_t currentTime{frc::Timer::GetFPGATimestamp()};
};
#endif
