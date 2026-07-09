

package frc.robot.superstructure;



import frc.robot.util.IState;


// Raymond: the reference splits this into two enums - Intention (what the driver requests) and InternalState (what the machine's actually running). you've collapsed both into one, which is exactly why SS needs the Flag hack to fake the request side. model it like the reference: separate Intention + InternalState.
public enum InternalStates implements IState {
    DISABLED, BOOT, IDLE, CLIMB1, CLIMB2, GRIPPING_CORAL1, GRIPPING_CORAL2, GRIPPING_ALGAE1, GRIPPING_ALGAE2, RELEASING, RAISING, LOWERING
}