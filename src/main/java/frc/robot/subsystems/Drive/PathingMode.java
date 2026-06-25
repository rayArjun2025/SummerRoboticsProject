// Raymond: lowercase package - frc.robot.subsystems.drive, rename the folder.
package frc.robot.subsystems.Drive;

import frc.robot.util.IState;

public enum PathingMode implements IState {
    DISABLED,
    FIELD_RELATIVE,
    POSE_FOLLOWING,
    // Raymond: reference has PATH_FOLLOWING and SHOOTING here, you replaced them with TRACKING. the TRACKING case in Drive does nothing, so this value is dead. line it back up with what Drive actually handles.
    TRACKING
}
