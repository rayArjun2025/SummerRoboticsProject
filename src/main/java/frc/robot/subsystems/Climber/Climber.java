// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: package is capitalized. all our packages are lowercase - this should be frc.robot.subsystems.climber. same for the folder name. rename the whole Climber/ folder to climber/.
package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs; // Raymond: you only need this because you're casting below. fix the inputs type and this import goes away.

import edu.wpi.first.wpilibj.util.Color8Bit; // Raymond: never used. you don't have a Climber2d mechanism so delete it.
import frc.robot.Constants;
import frc.robot.subsystems.Hand.HandStates; // Raymond: why does Climber import HandStates? it's never used. delete it.
import frc.robot.util.*; // Raymond: don't wildcard import. import the exact thing you use (StateMachineSubsystemBase) so it's obvious where stuff comes from.

// Raymond: run the formatter (spotlessApply). the whole class body is jammed against the margin and the field decl is stuck on the class line. it has to be readable before I can review it properly.
public class Climber extends StateMachineSubsystemBase<ClimberStates> {private static Climber instance;
private final ClimberIO io;
// Raymond: this should be ClimberIOInputsAutoLogged, not the raw inputs class. the AutoLogged one is what AdvantageKit logs and replays. as written replay won't work and you have to cast it later, which is the real reason you added that LoggableInputs import.
private final ClimberIO.ClimberIOInputs inputs = new ClimberIO.ClimberIOInputs();
// Raymond: name these with units like everywhere else - targetDegrees_deg/homeDegrees_deg. and 90/0 are magic numbers, put them in ClimberConstants.
private double targetDegrees=90.0;
private double homeDegrees=0.0;

// Raymond: singleton constructor has to be private. right now it's package-private so anything in the package can make a second Climber.
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
// Raymond: again, open the switch on its own line. don't cram it onto the method signature.
public void handleStateMachine() {switch (getState()) {
    case DISABLED:
        io.stopClimb();
        break;

    case IDLE:
        io.stopClimb();
        break;

    // Raymond: the comment says "do deep climb" but the state is SHALLOW_CLIMBING. pick one - which is it actually doing?
    case SHALLOW_CLIMBING: // do deep climb
        // Raymond: this condition is wrong. you check hook is ABOVE target (>) and wheel is BELOW target (<) - mixed directions, it'll basically never be true so it never goes IDLE. do what we do everywhere else: Math.abs(pos - target) < tolerance for both, and the tolerance is a constant, not 1.0 hardcoded. look at how reference climber uses isValueReached().
        if ((inputs.hookPositionDeg - targetDegrees) > 1.0 && (inputs.wheelPositionDeg - targetDegrees) < 1.0)
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(targetDegrees, targetDegrees);
        break;

    case RELEASING:
        // Raymond: same 1.0 magic number, make it a constant. and this only triggers when you're already below home, not when you're at it - use abs tolerance here too so it's consistent with the climb case.
        if (((inputs.hookPositionDeg - homeDegrees) <= 1.0) && ((inputs.wheelPositionDeg - homeDegrees) <= 1.0))
            queueState(ClimberStates.IDLE);
        else
            io.climbTo(homeDegrees, homeDegrees);
        break;

    // Raymond: WAITING does nothing and nothing ever queues it. either wire it up or drop it from ClimberStates - dead states just confuse people reading the enum.
    case WAITING:
        break;
        
    default:
        io.stopClimb();
        break;
    }
}

// Raymond: missing @Override - this overrides the base. and the cast to LoggableInputs is the giveaway you used the wrong inputs type. switch inputs to ClimberIOInputsAutoLogged and this becomes just Logger.processInputs("Climber", inputs), no cast.
public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climber", (LoggableInputs) inputs);
}
@Override
protected void outputPeriodic() {
    Logger.recordOutput("Climber/ClimbTargetDegrees", targetDegrees);
}


}