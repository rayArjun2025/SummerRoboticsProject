// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: lowercase package.
package frc.robot.subsystems.Climber;

import frc.robot.util.IState;

public enum ClimberStates implements IState {
    DISABLED,
    IDLE,
    SHALLOW_CLIMBING, // Raymond: name doesn't match the comment in Climber.java that says deep climb. and we usually split the motion into a TRAVELLING/HOLDING pair (see reference) so the climber holds position once it's there instead of re-commanding every loop. think about whether one CLIMBING state is enough.
    RELEASING,
    WAITING // Raymond: nothing ever queues this state. drop it until you actually use it.
}
