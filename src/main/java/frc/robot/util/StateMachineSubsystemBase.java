// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: big one. you have FOUR copies of this file - here in util, plus one each in subsystems/Elbow, Elevator, Shoulder. we keep exactly ONE shared base at subsystems/StateMachineSubsystemBase.java and every subsystem imports that. delete all four of these copies and point your subsystems at the real one, otherwise a fix to the base only lands in one place and the others rot.
// Raymond: and this copy is already behind - it's missing the MTimer the real base has (see notes below). so right now different subsystems are extending different versions of the "same" class. that's exactly the mess having one copy avoids.
package frc.robot.util;

import frc.robot.PerfTracker;

// Raymond: these two are in this same package (frc.robot.util) - you don't import your own package. delete both lines.
import frc.robot.util.IState;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

/** States are now handled inside the respective subsystem */
public abstract class StateMachineSubsystemBase<T extends Enum<T> & IState> implements IStateMachine<T> {

    private String name;
    // Raymond: missing the timer. real base has `protected final MTimer timer;` here so every subsystem can ask "how long have I been in this state". without it none of your state machines can time-gate anything.
    private T currentState;
    private boolean firstStep;

    public final T getState() {
        return currentState;
    }

    public final void queueState(T nextState) {
        if (currentState == null || !currentState.equals(nextState)) {
            currentState = nextState;
            // Raymond: real base does timer.reset() right here so the state clock starts over on every transition. you dropped it. add the timer field and this reset back.
            firstStep = true;
        } else {
            firstStep = false;
        }
    }

    public final boolean stateInit() {
        if (firstStep) {
            firstStep = false;
            return true;
        }
        return false;
    }

    public final boolean isState(T state) {
        return currentState.equals(state);
    }

    /** Creates a new StateMachineSubsystem. */
    public StateMachineSubsystemBase(String name) {
        this.name = name;
        firstStep = true;
        // Raymond: real base does `timer = new MTimer();` here. missing because you dropped the field.
    }

    protected void inputPeriodic() {}

    protected abstract void outputPeriodic();

    public final void periodic() {
        int id = PerfTracker.start(name);
        inputPeriodic();
        handleStateMachine();
        outputPeriodic();
        PerfTracker.end(id);
        Logger.recordOutput(name + "/State", currentState);
        // Raymond: real base also logs `Logger.recordOutput(name + "/StateTimer", timer.time());` here. you lost it with the timer. that StateTimer trace is how we debug stuck states in AdvantageScope - get it back.
    }
}