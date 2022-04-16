// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() = default;

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

void Drivetrain::JoystickDrive(double rightJoyX, double rightJoyY, double leftJoyY, double leftJoyX) {
    double rightOutput = -rightJoyY * DriveConstants::kJoystickMultiplier;
    double leftOutput = rightJoyY * DriveConstants::kJoystickMultiplier;

    double frontOutput = rightJoyX * DriveConstants::kJoystickMultiplier;
    double rearOutput = -rightJoyX * DriveConstants::kJoystickMultiplier;

    

    m_left.Set(TalonSRXControlMode::PercentOutput, rightOutput);
    m_right.Set(TalonSRXControlMode::PercentOutput, leftOutput);

    m_front.Set(TalonSRXControlMode::PercentOutput, frontOutput);
    m_rear.Set(TalonSRXControlMode::PercentOutput, rearOutput);
}
