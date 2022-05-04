// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <Constants.h>

#include <frc/ADIS16470_IMU.h>

class Drivetrain : public frc2::SubsystemBase {
 public:
  Drivetrain();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  void JoystickDrive(double rightJoyX, double rightJoyY, double leftJoyX, double leftJoyY);
  void FieldOrientedJoystickDrive(double rightJoyX, double rightJoyY, double leftJoyX, double leftJoyY);

  void ToggleFieldOriented();
  bool IsFieldOriented();

  void ResetIMU();
  units::degree_t GetIMUAngle();

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  WPI_TalonSRX m_front { CAN::kFrontMotorID };
  WPI_TalonSRX m_left { CAN::kLeftMotorID };
  WPI_TalonSRX m_right { CAN::kRightMotorID };
  WPI_TalonSRX m_rear { CAN::kRearMotorID };

  frc::ADIS16470_IMU m_imu { };

  bool fieldOriented = true;
};
