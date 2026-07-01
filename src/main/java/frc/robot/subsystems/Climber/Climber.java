// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import frc.robot.Constants;
import frc.robot.util.*;

public class Climber extends StateMachineSubsystemBase<ClimberStates> {private static Climber instance;
private final ClimberIO io;
private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();
private double targetDegrees=90.0;
private double homeDegrees=0.0;

Climber(ClimberIO io) {
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
public void handleStateMachine() {switch (getState()) {
    case DISABLED:
        io.stopClimb();
        break;

    case IDLE:
        io.stopClimb();
        break;

    case SHALLOW_CLIMBING:
        if ((inputs.hookPositionDeg - targetDegrees) > 1.0 && (inputs.wheelPositionDeg - targetDegrees) < 1.0)
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(targetDegrees, targetDegrees);
        break;

    case RELEASING:
        if (((inputs.hookPositionDeg - homeDegrees) <= 1.0) && ((inputs.wheelPositionDeg - homeDegrees) <= 1.0))
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(homeDegrees, homeDegrees);
        break;

    case WAITING:
        break;
        
    default:
        io.stopClimb();
        break;
    }
}

public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", (LoggableInputs) inputs);
}
@Override
protected void outputPeriodic() {
    Logger.recordOutput("Climber/ClimbTargetDegrees", targetDegrees);
}


}