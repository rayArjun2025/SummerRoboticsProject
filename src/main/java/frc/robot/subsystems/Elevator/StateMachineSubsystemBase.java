// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: this is a duplicate of the shared base. there's one StateMachineSubsystemBase in subsystems/ that everything extends - delete this copy and have Elevator extend that one. on top of being a dup, this copy is missing the MTimer/timer the real base has (timer.reset() in queueState, the StateTimer log in periodic), so any state that needs timing won't work. one base, no forks.
package frc.robot.subsystems.Elevator;

import frc.robot.PerfTracker;

import frc.robot.util.IState;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

/** States are now handled inside the respective subsystem */
public abstract class StateMachineSubsystemBase<T extends Enum<T> & IState> implements IStateMachine<T> {

    private String name;

    private T currentState;
    private boolean firstStep;

    public final T getState() {
        return currentState;
    }

    public final void queueState(T nextState) {
        if (currentState == null || !currentState.equals(nextState)) {
            currentState = nextState;
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
    }
}