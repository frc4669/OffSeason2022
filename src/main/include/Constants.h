// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace CAN {
    constexpr int kFrontMotorID = 2384; // PLACEHOLDER NUMBER
    constexpr int kRearMotorID = 2398; // PLACEHOLDER NUMBER
    constexpr int kLeftMotorID = 3248; // PLACEHOLDER NUMBER
    constexpr int kRightMotorID = 3894; // PLACEHOLDER NUMBER
}

namespace DriveConstants {
    constexpr double kJoystickMultiplier = 0.5;
}