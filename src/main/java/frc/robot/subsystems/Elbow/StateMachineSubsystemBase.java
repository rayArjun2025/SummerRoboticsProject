// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: delete this whole file. there's ONE shared base at
// frc.robot.subsystems.StateMachineSubsystemBase and every subsystem extends that - you do not copy
// it per folder. having a second copy means bug fixes to the real one never reach Elbow. just
// import the shared one.
// Raymond: and this copy is already stale: the real base has an MTimer that resets on every
// queueState and logs name + "/StateTimer" each loop. yours has neither, so Elbow loses the state
// timing we rely on. another reason to drop it.
package frc.robot.subsystems.Elbow;

import frc.robot.PerfTracker;
import frc.robot.util.IState;
import frc.robot.util.IStateMachine;
import org.littletonrobotics.junction.Logger;

/** States are now handled inside the respective subsystem */
public abstract class StateMachineSubsystemBase<T extends Enum<T> & IState>
    implements IStateMachine<T> {

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
