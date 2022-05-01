// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/RunCommand.h>

#include <controllers/GamepadF310.h>

#include <subsystems/Drivetrain.h>

#include "commands/ExampleCommand.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  Drivetrain* GetDrivetrain();

 private:
  // The robot's subsystems and commands are defined here...
  ExampleCommand m_autonomousCommand;

  Drivetrain m_drivetrain;

  GamepadF310 m_f310 { 0 };

  void ConfigureButtonBindings();
};
