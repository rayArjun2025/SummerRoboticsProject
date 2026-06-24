package frc.robot.subsystems.Drive;

import frc.robot.util.IState;

public enum PathingMode implements IState {
    DISABLED,
    FIELD_RELATIVE,
    POSE_FOLLOWING,
    TRACKING
}
