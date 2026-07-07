// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

// Raymond: run the formatter (spotlessApply). the whole class body is jammed against the margin and the field decl is stuck on the class line. it has to be readable before I can review it properly.
public class Climber extends StateMachineSubsystemBase<ClimberStates> {private static Climber instance;
private final ClimberIO io;
private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

private Climber(ClimberIO io) {
    super("Climber");
    this.io = io;
    queueState(ClimberStates.IDLE);
}

public static Climber getInstance() {
    if (instance == null) {
    switch (Constants.currentMode) {
        case REAL:
        instance = new Climber(new ClimberIOReal());
        break;

        case SIM:
        instance = new Climber(new ClimberIOSim());
        break;

        case REPLAY:
        instance = new Climber(new ClimberIO() {});
        break;
        default:
        break;
    }

    }
    return instance;
}

public void requestState(ClimberStates state) {
    queueState(state);
}

@Override
public void handleStateMachine() 
{
    switch (getState()) {
    case DISABLED:
        io.stopClimb();
        break;

    case IDLE:
        io.stopClimb();
        break;

    case SHALLOW_CLIMB_TRAVELLING:
        // Raymond: this condition is wrong. you check hook is ABOVE target (>) and wheel is BELOW target (<) - mixed directions, it'll basically never be true so it never goes IDLE. do what we do everywhere else: Math.abs(pos - target) < tolerance for both, and the tolerance is a constant, not 1.0 hardcoded. look at how reference climber uses isValueReached().
        if ((inputs.hookPositionDeg - ClimberConstants.targetDegrees_deg) > ClimberConstants.tolerance_deg && (inputs.wheelPositionDeg - ClimberConstants.targetDegrees_deg) < -ClimberConstants.tolerance_deg)
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(ClimberConstants.targetDegrees_deg, ClimberConstants.targetDegrees_deg);
        break;

    case HOLDING:
        io.stopClimb();
        break;

    case RELEASING:
        // Raymond: same 1.0 magic number, make it a constant. and this only triggers when you're already below home, not when you're at it - use abs tolerance here too so it's consistent with the climb case.
        if (((inputs.hookPositionDeg - ClimberConstants.homeDegrees_deg) <= ClimberConstants.tolerance_deg) && ((inputs.wheelPositionDeg - ClimberConstants.homeDegrees_deg) <= ClimberConstants.tolerance_deg))
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(ClimberConstants.homeDegrees_deg, ClimberConstants.homeDegrees_deg);
        break;
    
    default:
        io.stopClimb();
        break;
    }
}

@Override
public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", inputs);
}
@Override
protected void outputPeriodic() {
    Logger.recordOutput("Climber/ClimbTargetDegrees", ClimberConstants.targetDegrees_deg);
}


}