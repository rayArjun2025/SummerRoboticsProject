package frc.robot.subsystems.Hand;


import org.littletonrobotics.junction.Logger;

import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

public class Hand extends StateMachineSubsystemBase<HandStates> {private static Hand instance;

private final HandIO io;
private final HandIOInputsAutoLogged inputs =
    new HandIOInputsAutoLogged();
private double targetDegrees=90.0;
private double homeDegrees=0.0;

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
public void handleStateMachine() {switch (getState()) {
    case DISABLED:
        io.stopMoving();
        break;

    case IDLE:
        io.stopMoving();
        break;

    case GRIPPING_CORAL:
        if ((inputs.handPositionDeg - targetDegrees) <= 1.0) 
        queueState(HandStates.IDLE);
        else 
        io.grip(targetDegrees);
        
        break;
    case GRIPPING_ALGAE:
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

public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hand",  inputs);
}

@Override
protected void outputPeriodic() {
    Logger.recordOutput("Hand/TargetDegrees", targetDegrees);
}

}
