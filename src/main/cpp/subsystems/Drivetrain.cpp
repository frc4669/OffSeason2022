// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Drivetrain.h"

Drivetrain::Drivetrain() = default;

// This method will be called once per scheduler run
void Drivetrain::Periodic() {}

void Drivetrain::JoystickDrive(double rightJoyX, double rightJoyY, double leftJoyY, double leftJoyX) {
    double joyTurn = leftJoyX * DriveConstants::kJoystickMultiplier;

    double rightOutput = (-rightJoyY * DriveConstants::kJoystickMultiplier) + joyTurn; // Right motor output (oriented forward)
    rightOutput = std::clamp<double>(rightOutput, -1.0, 1.0);

    double leftOutput = (rightJoyY * DriveConstants::kJoystickMultiplier) + joyTurn; // Left motor output (oriented forward)
    leftOutput = std::clamp<double>(leftOutput, -1.0, 1.0);

    double frontOutput = (rightJoyX * DriveConstants::kJoystickMultiplier) + joyTurn; // Front motor output (oriented horizontally)
    frontOutput = std::clamp<double>(frontOutput, -1.0, 1.0);

    double rearOutput = (-rightJoyX * DriveConstants::kJoystickMultiplier) + joyTurn; // Rear motor output (oriented horizontally)
    rearOutput = std::clamp<double>(rearOutput, -1.0, 1.0);

    m_left.Set(TalonSRXControlMode::PercentOutput, rightOutput);
    m_right.Set(TalonSRXControlMode::PercentOutput, leftOutput);

    m_front.Set(TalonSRXControlMode::PercentOutput, frontOutput);
    m_rear.Set(TalonSRXControlMode::PercentOutput, rearOutput);
}
