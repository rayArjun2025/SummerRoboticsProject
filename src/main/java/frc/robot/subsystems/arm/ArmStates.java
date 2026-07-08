package frc.robot.subsystems.arm;

import frc.robot.util.IState;

public enum ArmStates implements IState{
    TRAVELLING_TO_POSITION,
    HOLDING_POSITION,
    IDLE,
    DISABLED

}
