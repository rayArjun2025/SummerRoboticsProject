package frc.robot.subsystems.elbow;

import frc.robot.util.IState;

public enum ElbowStates implements IState{
    INCREASING_ELEVATION_ANGLE, 
    DECREASING_ELEVATION_ANGLE,
    IDLE,
    DISABLED

}