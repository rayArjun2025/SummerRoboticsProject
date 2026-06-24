// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.IState;


public enum SuperstructureStates implements IState {
    STOWED,
    INTAKE_CORAL,
    SCORE,
    CLIMBING,
}
