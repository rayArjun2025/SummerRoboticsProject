// Raymond: lowercase package - frc.robot.subsystems.elbow.
package frc.robot.subsystems.Elbow;

import frc.robot.util.IState;

// Raymond: no space before the brace - spotlessApply.
public enum ElbowStates implements IState {
  // Raymond: these are an elbow, not an elevator - "ELEVATION_ANGLE" is confusing. and there's no
  // DISABLED state, every other subsystem has one so you can kill the motor on disable. add it and
  // handle it.
  INCREASING_ELEVATION_ANGLE,
  DECREASING_ELEVATION_ANGLE,
  IDLE
}
