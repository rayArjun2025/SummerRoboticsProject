package frc.robot.subsystems.hand;


import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

// Raymond: run spotlessApply. class is jammed on the margin and the field decl is on the class line - exact same thing I flagged in Climber.java, go look at the cleanup notes there, all of it applies here too.
public class Hand extends StateMachineSubsystemBase<HandStates> {private static Hand instance;

private final HandIO io;
private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();
private double targetDegrees = 0.0;

// Raymond: private constructor for a singleton.
Hand(HandIO io) {
    super("Hand");
    this.io = io;
    queueState(HandStates.IDLE);
}

public static Hand getInstance() {
    if (instance == null) {
        switch (Constants.currentMode) {
            case SIM:
                instance = new Hand(new HandIOSim());
                break;
            case REAL:
                instance = new Hand(new HandIOReal());
                break;
            default:
                instance = new Hand(new HandIO() {});
                break;
        }
    }
    return instance;
}

public void requestState(HandStates state) {
    queueState(state);
}

@Override
// Raymond: switch on its own line.
public void handleStateMachine() {switch (getState()) {
    case DISABLED:
        io.stopMoving();
        break;

    case IDLE:
        io.stopMoving();
        break;

    // Raymond: brace your if/else bodies. unbraced one-liners stacked like this are a bug waiting to happen the moment someone adds a second line. and indent them.
    case GRIPPING_CORAL:
        // Raymond: CORAL exits when pos-target <= 1.0 but ALGAE (right below) exits when pos-target > 1.0 - opposite conditions for what should be the same "did we reach the target" check. one of these is wrong. use Math.abs(pos - target) < TOLERANCE_DEG for both, with the tolerance as a constant instead of 1.0.
        if (Math.abs(inputs.handPositionDeg - HandConstants.coralTarget_deg) <= HandConstants.tolerance_deg) {
            queueState(HandStates.IDLE);
        } 
        else {
            io.grip(HandConstants.coralTarget_deg);
            targetDegrees = HandConstants.coralTarget_deg;
        }
        break;

    case GRIPPING_ALGAE:
        if (Math.abs(inputs.handPositionDeg - HandConstants.algaeTarget_deg) <= HandConstants.tolerance_deg) {
        queueState(HandStates.IDLE);
    }
        else {
            io.grip(HandConstants.algaeTarget_deg);
            targetDegrees = HandConstants.algaeTarget_deg; 
        }
        break;

    case RELEASING:
        if (Math.abs(inputs.handPositionDeg - HandConstants.home_deg) <= HandConstants.tolerance_deg) {
        queueState(HandStates.IDLE);
        }
        else {
        io.grip(HandConstants.home_deg);
        }
        break;
        
    default:
        io.stopMoving();
        targetDegrees = HandConstants.home_deg;
        break;
    }
}

@Override
public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hand", inputs);
}

@Override
protected void outputPeriodic() {
    Logger.recordOutput("Hand/TargetDegrees", targetDegrees);
}

}
