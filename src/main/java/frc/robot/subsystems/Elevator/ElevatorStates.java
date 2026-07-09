package frc.robot.subsystems.elevator;


import frc.robot.util.IState;
public enum ElevatorStates implements IState {
    TRAVELLING,
    HOLDING,
    IDLE,
    DISABLED
}