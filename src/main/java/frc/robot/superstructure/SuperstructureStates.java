// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

// Raymond: SubsystemBase is never used here, delete it.
import frc.robot.util.IState;

// Raymond: the reference splits this into two enums - Intention (what the driver requests) and
// InternalState (what the machine's actually running). you've collapsed both into one, which is
// exactly why SS needs the Flag hack to fake the request side. model it like the reference:
// separate Intention + InternalState.
public enum SuperstructureStates implements IState {
  STOWED,
  // Raymond: INTAKE_CORAL is never queued anywhere in SS - dead state. wire it up or remove it.
  INTAKE_CORAL,
  SCORE,
  CLIMBING,
}
