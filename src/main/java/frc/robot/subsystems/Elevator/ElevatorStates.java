package frc.robot.subsystems.elevator;


import frc.robot.util.IState;
public enum ElevatorStates implements IState {
    MOVING_UP,
    MOVING_DOWN,
    IDLE,
    DISABLED
}