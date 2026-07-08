// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
package frc.robot.subsystems.Shoulder;

import frc.robot.util.IState;

// Raymond: name it ShoulderStates (plural) - every other state enum is XxxStates, see
// ClimberStates/HandStates. and there's no DISABLED state to stop the motor, Shoulder.java needs
// one.
public enum ShoulderState implements IState {
  INCREASE_SHOOTING_ANGLE,
  DECREASE_SHOOTING_ANGLE,
  IDLE
}
