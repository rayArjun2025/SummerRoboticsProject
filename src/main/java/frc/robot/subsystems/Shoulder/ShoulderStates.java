package frc.robot.subsystems.shoulder;

import frc.robot.util.IState;

public enum ShoulderStates implements IState{
    TRAVELLING_TO_POSITION,
    HOLDING_POSITION,
    IDLE,
    DISABLED

}
