// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;
import frc.robot.util.IState;

public enum DriveStates implements IState {
    DISABLED,
    IDLE,
    MOVING
}