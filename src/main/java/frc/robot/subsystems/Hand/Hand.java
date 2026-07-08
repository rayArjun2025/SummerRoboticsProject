package frc.robot.subsystems.hand;

import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;
import org.littletonrobotics.junction.Logger;

// Raymond:
// - exact same thing I flagged in Climber.java, go look at the cleanup notes there, all of it
// applies here too.
public class Hand extends StateMachineSubsystemBase<HandStates> {
  private static Hand instance;

  private final HandIO io;
  private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();
  private double targetDegrees = 0.0;

  private Hand(HandIO io) {
    super("Hand");
    this.io = io;
    queueState(HandStates.GRIPPING_CORAL);
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
  public void handleStateMachine() {
    switch (getState()) {
      case DISABLED:
        io.stopMoving();
        break;

      case IDLE:
        io.stopMoving();
        break;

      case GRIPPING_CORAL:
        if (isValueReached(inputs.handPositionDeg, HandConstants.coralTarget_deg, HandConstants.tolerance_deg)) {
          queueState(HandStates.IDLE);
        } else {
          io.grip(HandConstants.coralTarget_deg);
          targetDegrees = HandConstants.coralTarget_deg;
        }
        break;

      case GRIPPING_ALGAE:
        if (isValueReached(inputs.handPositionDeg, HandConstants.algaeTarget_deg, HandConstants.tolerance_deg)) {
          queueState(HandStates.IDLE);
        } 
        else {
          io.grip(HandConstants.algaeTarget_deg);
          targetDegrees = HandConstants.algaeTarget_deg;
        }
        break;

      case RELEASING:
        if (isValueReached(inputs.handPositionDeg, HandConstants.home_deg, HandConstants.tolerance_deg)) {
          queueState(HandStates.IDLE);
        } else {
          io.grip(HandConstants.home_deg);
        }
        break;

      case HOLDING:
        if (!isValueReached(inputs.handPositionDeg, targetDegrees, HandConstants.tolerance_deg)) {
          queueState(HandStates.IDLE);
        } 
        else {
          io.grip(targetDegrees);
        }
        break;

      case HOMING:
        if (isValueReached(inputs.handPositionDeg, HandConstants.home_deg, HandConstants.tolerance_deg)) {
          io.zeroPosition();
          io.stopMoving();
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

  public boolean isValueReached(double position_deg, double target_deg, double tolerance_deg) {
    return (Math.abs(position_deg - target_deg) <= tolerance_deg);
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
