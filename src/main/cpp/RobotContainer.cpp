// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

RobotContainer::RobotContainer() : m_autonomousCommand(&m_subsystem) {
  // Initialize all of your commands and subsystems here

  m_drivetrain.SetDefaultCommand(frc2::RunCommand(
    [this] { m_drivetrain.JoystickDrive(m_f310.getRightJoyX(), m_f310.getRightJoyY(), m_f310.getLeftJoyY(), m_f310.getLeftJoyX()); },
    { &m_drivetrain }
  ));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}
