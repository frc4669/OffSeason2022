// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() {
  m_imu.Calibrate();  // Calibrates the IMU, takes a few seconds -- important to not move robot
}

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

void Drivetrain::ResetIMU() {
  m_imu.Reset();
}

units::degree_t Drivetrain::GetIMUAngle() {
  return m_imu.GetAngle();
}

void Drivetrain::JoystickDrive(double rightJoyX, double rightJoyY, double leftJoyY, double leftJoyX) {
  /*frc::SmartDashboard::PutNumber("RIGHT X", rightJoyX);
  frc::SmartDashboard::PutNumber("RIGHT Y", rightJoyY);
  frc::SmartDashboard::PutNumber("LEFT X", leftJoyX);
  frc::SmartDashboard::PutNumber("LEFT Y", leftJoyY);
  frc::SmartDashboard::PutNumber("TURN", leftJoyX * DriveConstants::kJoystickMultiplier);*/

  double joyTurn = leftJoyX * DriveConstants::kTurnMultiplier;

  double rightOutput = (-rightJoyY * DriveConstants::kMoveMultiplier) + joyTurn; // Right motor output (oriented forward)
  rightOutput = std::clamp<double>(rightOutput, -1.0, 1.0);

  double leftOutput = (rightJoyY * DriveConstants::kMoveMultiplier) + joyTurn; // Left motor output (oriented forward)
  leftOutput = std::clamp<double>(leftOutput, -1.0, 1.0);

  double frontOutput = (rightJoyX * DriveConstants::kMoveMultiplier) + joyTurn; // Front motor output (oriented horizontally)
  frontOutput = std::clamp<double>(frontOutput, -1.0, 1.0);

  double rearOutput = (-rightJoyX * DriveConstants::kMoveMultiplier) + joyTurn; // Rear motor output (oriented horizontally)
  rearOutput = std::clamp<double>(rearOutput, -1.0, 1.0);

  frc::SmartDashboard::PutNumber("FRONT OUT", frontOutput);
  frc::SmartDashboard::PutNumber("REAR OUT", rearOutput);
  frc::SmartDashboard::PutNumber("LEFT OUT", leftOutput);
  frc::SmartDashboard::PutNumber("RIGHT OUT", rightOutput);

  m_left.Set(TalonSRXControlMode::PercentOutput, rightOutput);
  m_right.Set(TalonSRXControlMode::PercentOutput, leftOutput);

  m_front.Set(TalonSRXControlMode::PercentOutput, frontOutput);
  m_rear.Set(TalonSRXControlMode::PercentOutput, rearOutput);
}

void Drivetrain::FieldOrientedJoystickDrive(double rightJoyX, double rightJoyY, double leftJoyX, double leftJoyY) {
  double IMUAngle = units::radian_t(GetIMUAngle()).value();

  double joystickAngle = atan2(rightJoyY, rightJoyX);
  double hypotenuse = hypot(rightJoyX, rightJoyY);

  double newAngle = joystickAngle - IMUAngle;

  double newX = hypotenuse * cos(newAngle);
  double newY = hypotenuse * sin(newAngle);

  JoystickDrive(newX, newY, leftJoyX, leftJoyY);
}

void Drivetrain::ToggleFieldOriented() {
  fieldOriented = !fieldOriented;
}

bool Drivetrain::IsFieldOriented() {
  return fieldOriented;
}
