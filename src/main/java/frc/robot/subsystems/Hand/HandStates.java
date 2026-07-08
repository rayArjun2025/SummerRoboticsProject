package frc.robot.subsystems.hand;

import frc.robot.util.IState;

public enum HandStates implements IState {
  DISABLED,
  IDLE,
  GRIPPING_CORAL,
  GRIPPING_ALGAE,
  RELEASING,
  HOLDING,
  HOMING
}
