package frc.robot.subsystems.Hand;


import frc.robot.util.IState;
public enum HandStates implements IState {
    DISABLED,
    IDLE,
    GRIPPING_CORAL,
    GRIPPING_ALGAE,
    RELEASING
}