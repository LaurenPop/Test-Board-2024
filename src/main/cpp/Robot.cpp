/*
 * Team 5561 2022 Code
 *
 * This code runs the 2020 robot which is capable of the following:
 * - Swerve Drive (beta 02/10/2020)
 * - Shooting balls without vision (beta 02/26/2022)
 * - Climber active (beta 02/26/2022)
 *
 * */
using namespace ctre::phoenix6;

#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/DriverStation.h>
#include "Lookup.hpp"
//#include "Encoders.hpp"
//#include "Gyro.hpp"
//#include "IO_Sensors.hpp"
//#include "Driver_inputs.hpp"
//#include "Odometry.hpp"
//#include "DriveControl.hpp"
//#include "Manipulator.hpp"
//#include "BallHandler.hpp"
//#include "LightControl.hpp"
// #include "VisionV2.hpp"
//#include "ADAS.hpp"
//#include "ADAS_BT.hpp"
//#include "ADAS_DM.hpp"
//#include "ADAS_MN.hpp"
//#include <ctime>
/*
T_RobotState VeROBO_e_RobotState = E_Init;
frc::DriverStation::Alliance VeROBO_e_AllianceColor = frc::DriverStation::Alliance::kBlue;
double VeROBO_t_MatchTimeRemaining = 0;
bool VeROBO_b_TestState = false;
double vROBO_t_MotorTime = 0;
bool vROBO_b_MotorPower = false;

double Chris_MTR = 0;
double CHRIS_Second = 0;
float SparkSpeedPID = 0;*/
double V_M1_Speed = 0;
double V_M2_Speed = 0;
double V_KrakenTest_Speed = 0;
double V_LauncherPID_Gx[E_PID_SparkMaxCalSz];
// bool FakeButton;

constexpr units::time::second_t print_period{500_ms};

/******************************************************************************
 * Function:     RobotMotorCommands
 *
 * Description:  Contains the outputs for the motors.
 ******************************************************************************/
void Robot::RobotMotorCommands()
{

}

/******************************************************************************
 * Function:     RobotInit
 *
 * Description:  Called during initialization of the robot.
 ******************************************************************************/
void Robot::RobotInit()
{
  //VeROBO_t_MatchTimeRemaining = frc::Timer::GetMatchTime().value();

  //  start with factory-default configs
//    ctre::phoenix6::configs::MotorOutputConfigs currentConfigs{};

//   currentConfigs.Inverted = ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive;

//  m_krakentest.GetConfigurator().Apply(currentConfigs);

  /* Configure CANcoder */
  ctre::phoenix6::configs::CANcoderConfiguration toApply{};

  /* User can change the configs if they want, or leave it empty for factory-default */

  cancoder.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  cancoder.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder.GetVelocity().SetUpdateFrequency(100_Hz);

  m_Motor1_PID.SetP(K_BH_LauncherPID_Gx[E_kP]);
  m_Motor1_PID.SetI(K_BH_LauncherPID_Gx[E_kI]);
  m_Motor1_PID.SetD(K_BH_LauncherPID_Gx[E_kD]);
  m_Motor1_PID.SetIZone(K_BH_LauncherPID_Gx[E_kIz]);
  m_Motor1_PID.SetFF(K_BH_LauncherPID_Gx[E_kFF]);
  m_Motor1_PID.SetOutputRange(K_BH_LauncherPID_Gx[E_kMinOutput], K_BH_LauncherPID_Gx[E_kMaxOutput]);
  m_Motor2_PID.SetP(K_BH_LauncherPID_Gx[E_kP]);
  m_Motor2_PID.SetI(K_BH_LauncherPID_Gx[E_kI]);
  m_Motor2_PID.SetD(K_BH_LauncherPID_Gx[E_kD]);
  m_Motor2_PID.SetIZone(K_BH_LauncherPID_Gx[E_kIz]);
  m_Motor2_PID.SetFF(K_BH_LauncherPID_Gx[E_kFF]);
  m_Motor2_PID.SetOutputRange(K_BH_LauncherPID_Gx[E_kMinOutput], K_BH_LauncherPID_Gx[E_kMaxOutput]);

  frc::SmartDashboard::PutNumber("P Gain", K_BH_LauncherPID_Gx[E_kP]);
  frc::SmartDashboard::PutNumber("I Gain", K_BH_LauncherPID_Gx[E_kI]);
  frc::SmartDashboard::PutNumber("D Gain", K_BH_LauncherPID_Gx[E_kD]);
  frc::SmartDashboard::PutNumber("I Zone", K_BH_LauncherPID_Gx[E_kIz]);
  frc::SmartDashboard::PutNumber("Feed Forward", K_BH_LauncherPID_Gx[E_kFF]);
  frc::SmartDashboard::PutNumber("Max Output", K_BH_LauncherPID_Gx[E_kMaxOutput]);
  frc::SmartDashboard::PutNumber("Min Output", K_BH_LauncherPID_Gx[E_kMinOutput]);

  // display secondary coefficients
  frc::SmartDashboard::PutNumber("M1 Speed", 0);
  frc::SmartDashboard::PutNumber("M2 Speed", 0);
  frc::SmartDashboard::PutNumber("M1 Speed Measured", 0);
  frc::SmartDashboard::PutNumber("M2 Speed Measured", 0);

  frc::SmartDashboard::PutNumber("Ramp Rate", 6);

  frc::SmartDashboard::PutNumber("M1 Desired", 0);
  frc::SmartDashboard::PutNumber("M2 Desired", 0);

  frc::SmartDashboard::PutNumber("Kraken Desired", 0);

}

/******************************************************************************
 * Function:     RobotPeriodic
 *
 * Description:  Function called periodically (not defined well as to what
 *               "periodically" means).
 ******************************************************************************/
void Robot::RobotPeriodic()
{

}

/******************************************************************************
 * Function:     AutonomousInit
 *S
 * Description:  Function called at init while in autonomous.  This is where we
 *               should zero out anything that we need to before autonomous mode.
 ******************************************************************************/
void Robot::AutonomousInit()
{

}

/******************************************************************************
 * Function:     AutonomousPeriodic
 *
 * Description:  Function called periodically in autonomous.  This is where we
 *               should place our primary autonomous control code.
 ******************************************************************************/
void Robot::AutonomousPeriodic()
{

}

/******************************************************************************
 * Function:     TeleopInit
 *
 * Description:  Function called when starting out in teleop mode.
 *               We should zero out all of our global varibles.
 ******************************************************************************/
void Robot::TeleopInit()
{

}

/******************************************************************************
 * Function:     TeleopPeriodic
 *
 * Description:  Primary function called when in teleop mode.
 ******************************************************************************/
void Robot::TeleopPeriodic()
{

  double L_DesiredSpeed1 = 0;
  double L_DesiredSpeed2 = 0;
  double L_Krakenspeed1 = 0;
  
  double L_p = frc::SmartDashboard::GetNumber("P Gain", K_BH_LauncherPID_Gx[E_kP]);
  double L_i = frc::SmartDashboard::GetNumber("I Gain", K_BH_LauncherPID_Gx[E_kI]);
  double L_d = frc::SmartDashboard::GetNumber("D Gain", K_BH_LauncherPID_Gx[E_kD]);
  double L_iz = frc::SmartDashboard::GetNumber("I Zone", K_BH_LauncherPID_Gx[E_kIz]);
  double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", K_BH_LauncherPID_Gx[E_kFF]);
  double L_max = frc::SmartDashboard::GetNumber("Max Output", K_BH_LauncherPID_Gx[E_kMaxOutput]);
  double L_min = frc::SmartDashboard::GetNumber("Min Output", K_BH_LauncherPID_Gx[E_kMinOutput]);
  
  double L_Ramp = frc::SmartDashboard::GetNumber("Ramp Rate", 0);

  L_DesiredSpeed1 = frc::SmartDashboard::GetNumber("M1 Speed", 0);
  L_DesiredSpeed2 = frc::SmartDashboard::GetNumber("M2 Speed", 0);

// Shuffleboard speed inputs along with ramp to time.
  V_M1_Speed = RampTo(L_DesiredSpeed1, V_M1_Speed, L_Ramp);
  V_M2_Speed = RampTo(L_DesiredSpeed2, V_M2_Speed, L_Ramp);
  V_KrakenTest_Speed = RampTo(L_Krakenspeed1, V_KrakenTest_Speed, L_Ramp);

  if((L_p != V_LauncherPID_Gx[E_kP]))   { m_Motor1_PID.SetP(L_p); m_Motor2_PID.SetP(L_p); V_LauncherPID_Gx[E_kP] = L_p; }
  if((L_i != V_LauncherPID_Gx[E_kI]))   { m_Motor1_PID.SetI(L_i); m_Motor2_PID.SetI(L_i); V_LauncherPID_Gx[E_kI] = L_i; }
  if((L_d != V_LauncherPID_Gx[E_kD]))   { m_Motor1_PID.SetD(L_d); m_Motor2_PID.SetD(L_d); V_LauncherPID_Gx[E_kD] = L_d; }
  if((L_iz != V_LauncherPID_Gx[E_kIz])) { m_Motor1_PID.SetIZone(L_iz); m_Motor2_PID.SetIZone(L_iz); V_LauncherPID_Gx[E_kIz] = L_iz; }
  if((L_ff != V_LauncherPID_Gx[E_kFF])) { m_Motor1_PID.SetFF(L_ff); m_Motor2_PID.SetFF(L_ff); V_LauncherPID_Gx[E_kFF] = L_ff; }
  if((L_max != V_LauncherPID_Gx[E_kMaxOutput]) || (L_min != K_BH_LauncherPID_Gx[E_kMinOutput])) { m_Motor1_PID.SetOutputRange(L_min, L_max); m_Motor2_PID.SetOutputRange(L_min, L_max); V_LauncherPID_Gx[E_kMinOutput] = L_min; V_LauncherPID_Gx[E_kMaxOutput] = L_max; }

  m_Motor1_PID.SetReference(V_M1_Speed, rev::CANSparkMax::ControlType::kVelocity);
  m_Motor2_PID.SetReference(V_M2_Speed, rev::CANSparkMax::ControlType::kVelocity);

  frc::SmartDashboard::PutNumber("M1 Speed Measured", m_Motor1Encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("M2 Speed Measured", m_Motor2Encoder.GetVelocity()); 

  frc::SmartDashboard::PutNumber("M1 Desired", V_M1_Speed);
  frc::SmartDashboard::PutNumber("M2 Desired", V_M2_Speed);
  frc::SmartDashboard::PutNumber("Kraken Desired", V_KrakenTest_Speed);
}

/******************************************************************************
 * Function:     TestPeriodic
 *
 * Description:  Called during the test phase initiated on the driver station.
 ******************************************************************************/
void Robot::TestPeriodic()
{

  double L_p = frc::SmartDashboard::GetNumber("P Gain", K_BH_LauncherPID_Gx[E_kP]);
  double L_i = frc::SmartDashboard::GetNumber("I Gain", K_BH_LauncherPID_Gx[E_kI]);
  double L_d = frc::SmartDashboard::GetNumber("D Gain", K_BH_LauncherPID_Gx[E_kD]);
  double L_iz = frc::SmartDashboard::GetNumber("I Zone", K_BH_LauncherPID_Gx[E_kIz]);
  double L_ff = frc::SmartDashboard::GetNumber("Feed Forward", K_BH_LauncherPID_Gx[E_kFF]);
  double L_max = frc::SmartDashboard::GetNumber("Max Output", K_BH_LauncherPID_Gx[E_kMaxOutput]);
  double L_min = frc::SmartDashboard::GetNumber("Min Output", K_BH_LauncherPID_Gx[E_kMinOutput]);

  V_M1_Speed = frc::SmartDashboard::GetNumber("M1 Speed", 0);
  V_M2_Speed = frc::SmartDashboard::GetNumber("M2 Speed", 0);

  if((L_p != V_LauncherPID_Gx[E_kP]))   { m_Motor1_PID.SetP(L_p); m_Motor1_PID.SetP(L_p); V_LauncherPID_Gx[E_kP] = L_p; }
  if((L_i != V_LauncherPID_Gx[E_kI]))   { m_Motor1_PID.SetI(L_i); m_Motor1_PID.SetI(L_i); V_LauncherPID_Gx[E_kI] = L_i; }
  if((L_d != V_LauncherPID_Gx[E_kD]))   { m_Motor1_PID.SetD(L_d); m_Motor1_PID.SetD(L_d); V_LauncherPID_Gx[E_kD] = L_d; }
  if((L_iz != V_LauncherPID_Gx[E_kIz])) { m_Motor1_PID.SetIZone(L_iz); m_Motor1_PID.SetIZone(L_iz); V_LauncherPID_Gx[E_kIz] = L_iz; }
  if((L_ff != V_LauncherPID_Gx[E_kFF])) { m_Motor1_PID.SetFF(L_ff); m_Motor1_PID.SetFF(L_ff); V_LauncherPID_Gx[E_kFF] = L_ff; }
  if((L_max != V_LauncherPID_Gx[E_kMaxOutput]) || (L_min != K_BH_LauncherPID_Gx[E_kMinOutput])) { m_Motor1_PID.SetOutputRange(L_min, L_max); m_Motor1_PID.SetOutputRange(L_min, L_max); V_LauncherPID_Gx[E_kMinOutput] = L_min; V_LauncherPID_Gx[E_kMaxOutput] = L_max; }

  m_Motor1_PID.SetReference(V_M1_Speed, rev::CANSparkMax::ControlType::kVelocity);
  m_Motor2_PID.SetReference(V_M2_Speed, rev::CANSparkMax::ControlType::kVelocity);

  frc::SmartDashboard::PutNumber("M1 Speed", m_Motor1Encoder.GetVelocity());
  frc::SmartDashboard::PutNumber("M2 Speed", m_Motor2Encoder.GetVelocity()); 
  // frc::SmartDashboard::PutNumber("Kraken Speed". m_)
}

#ifndef RUNNING_FRC_TESTS
/******************************************************************************
 * Function:     main
 *
 * Description:  This is the main calling function for the robot.
 ******************************************************************************/
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
