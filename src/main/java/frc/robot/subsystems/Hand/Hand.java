// Raymond: lowercase package - frc.robot.subsystems.hand. rename the folder too.
package frc.robot.subsystems.Hand;


import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs; // Raymond: only needed because of the cast below. fix the inputs type and drop this.

import edu.wpi.first.wpilibj.util.Color8Bit; // Raymond: unused, delete.
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

// Raymond: run spotlessApply. class is jammed on the margin and the field decl is on the class line - exact same thing I flagged in Climber.java, go look at the cleanup notes there, all of it applies here too.
public class Hand extends StateMachineSubsystemBase<HandStates> {private static Hand instance;

private final HandIO io;
// Raymond: use HandIOInputsAutoLogged here, not the raw inputs class - same reason as Climber. you even wrote the AutoLogged class and didn't use it.
private final HandIO.HandIOInputs inputs =
    new HandIO.HandIOInputs();
// Raymond: units in the name (_deg) and these belong in HandConstants, not magic 90/0 here.
private double targetDegrees=90.0;
private double homeDegrees=0.0;

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
        if ((inputs.handPositionDeg - targetDegrees) <= 1.0)
        queueState(HandStates.IDLE);
        else
        io.grip(targetDegrees);

        break;
    case GRIPPING_ALGAE:
        // Raymond: also both coral and algae grip to the SAME targetDegrees - so what's the actual difference between gripping coral vs algae? if they need different grip positions that has to come from somewhere. right now these two states are identical except for the broken condition.
        if ((inputs.handPositionDeg - targetDegrees) > 1.0)
        queueState(HandStates.IDLE);
        else
        io.grip(targetDegrees);

        break;

    case RELEASING:
        if ((inputs.handPositionDeg - homeDegrees) <= 1.0)
        queueState(HandStates.IDLE);
        else
        io.grip(homeDegrees);
        break;
        
    default:
        io.stopMoving();
        break;
    }
}

// Raymond: needs @Override, and drop the (LoggableInputs) cast once inputs is the AutoLogged type.
public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hand", (LoggableInputs) inputs);
}

@Override
protected void outputPeriodic() {
    Logger.recordOutput("Hand/TargetDegrees", targetDegrees);
}

}
