// Raymond: lowercase package - frc.robot.subsystems.drive (rename the folder). otherwise a clean
// copy of the reference, nothing else here.
package frc.robot.subsystems.drive;

import frc.robot.util.IState;

public enum PathingOverride implements IState {
  NONE,
  INTAKING,
  TRACKING,
  BASELOCK,
  SHOOTING
}
