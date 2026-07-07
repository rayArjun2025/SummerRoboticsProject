// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import frc.robot.util.IState;

public enum ClimberStates implements IState {
    DISABLED,
    IDLE,
    SHALLOW_CLIMB_TRAVELLING,
    HOLDING,
    RELEASING,
}
