

package frc.robot.superstructure;



import frc.robot.util.IState;


// Raymond: the reference splits this into two enums - Intention (what the driver requests) and InternalState (what the machine's actually running). you've collapsed both into one, which is exactly why SS needs the Flag hack to fake the request side. model it like the reference: separate Intention + InternalState.
public enum IntentionStates implements IState {
    IDLE, CLIMB, GRIPPING_CORAL, GRIPPING_ALGAE, RELEASING, RAISING, LOWERING
}