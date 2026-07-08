// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import frc.robot.Constants;

import frc.robot.util.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Climber extends StateMachineSubsystemBase<ClimberStates> {
  private static Climber instance;
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
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        io.stopClimb();
        break;

      case IDLE:
        io.stopClimb();
        break;

      case SHALLOW_CLIMB_TRAVELLING:
        if (isValueReached(inputs.hookPositionDeg, ClimberConstants.targetDegrees_deg, ClimberConstants.tolerance_deg) && 
        isValueReached(inputs.wheelPositionDeg, ClimberConstants.targetDegrees_deg, ClimberConstants.tolerance_deg)) {
              queueState(ClimberStates.HOLDING);
            }

        else {
          io.climbTo(ClimberConstants.targetDegrees_deg, ClimberConstants.targetDegrees_deg);
        }
        break;

      case HOLDING:
        if (!isValueReached(inputs.hookPositionDeg, ClimberConstants.targetDegrees_deg, ClimberConstants.tolerance_deg) || 
        !isValueReached(inputs.wheelPositionDeg, ClimberConstants.targetDegrees_deg, ClimberConstants.tolerance_deg)) {
              queueState(ClimberStates.SHALLOW_CLIMB_TRAVELLING);
            }
        else {
          io.climbTo(ClimberConstants.targetDegrees_deg, ClimberConstants.targetDegrees_deg);
        }
        break;

      case RELEASING:
        if (isValueReached(inputs.hookPositionDeg, ClimberConstants.homeDegrees_deg, ClimberConstants.tolerance_deg) && 
        isValueReached(inputs.wheelPositionDeg, ClimberConstants.homeDegrees_deg, ClimberConstants.tolerance_deg)) { 
          queueState(ClimberStates.IDLE);
        }
        else {
          io.climbTo(ClimberConstants.homeDegrees_deg, ClimberConstants.homeDegrees_deg); 
        }
        break;

      case HOMING:
        if (isValueReached(inputs.hookPositionDeg, ClimberConstants.homeDegrees_deg, ClimberConstants.tolerance_deg) && 
        isValueReached(inputs.wheelPositionDeg, ClimberConstants.homeDegrees_deg, ClimberConstants.tolerance_deg)) { 
          io.zeroPosition();
          io.stopClimb();
          queueState(ClimberStates.IDLE);
        }
        else {
          io.climbTo(ClimberConstants.homeDegrees_deg, ClimberConstants.homeDegrees_deg);
        }
      default:
        io.stopClimb();
        break;
    }
  }

  public boolean isValueReached(double position_deg, double target_deg, double tolerance_deg) {
    return (Math.abs(position_deg - target_deg) <= tolerance_deg);
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
