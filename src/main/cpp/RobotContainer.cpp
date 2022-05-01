// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <commands/ResetIMU.h>

RobotContainer::RobotContainer() {
  m_drivetrain.SetDefaultCommand(frc2::RunCommand(
    [this] { m_drivetrain.FieldOrientedJoystickDrive(m_f310.getRightJoyX(), m_f310.getRightJoyY(), m_f310.getLeftJoyY(), m_f310.getLeftJoyX()); },
    { &m_drivetrain }
  ));

  // Configure the button bindings
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {
  // Configure your button bindings here

  m_f310.redButton.WhenPressed( ResetIMU(&m_drivetrain) );
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // An example command will be run in autonomous
  return &m_autonomousCommand;
}

Drivetrain* RobotContainer::GetDrivetrain() {
  return &m_drivetrain;
}