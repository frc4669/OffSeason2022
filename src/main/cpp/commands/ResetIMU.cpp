// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ResetIMU.h"

ResetIMU::ResetIMU(Drivetrain* drivetrain) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({ drivetrain });
  this->drivetrain = drivetrain;
}

// Called when the command is initially scheduled.
void ResetIMU::Initialize() {
  drivetrain->ResetIMU();
}

// Called repeatedly when this Command is scheduled to run
void ResetIMU::Execute() {}

// Called once the command ends or is interrupted.
void ResetIMU::End(bool interrupted) {}

// Returns true when the command should end.
bool ResetIMU::IsFinished() {
  return true; // not sure if this is how i make the command instantaneously end after initialize()
}
