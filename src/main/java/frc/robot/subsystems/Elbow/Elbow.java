// Raymond: package is capitalized. all our packages are lowercase - this should be
// frc.robot.subsystems.elbow, and rename the Elbow/ folder to elbow/ too. same thing I flagged on
// Climber and Hand.
package frc.robot.subsystems.Elbow;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// Raymond: missing space before the brace - run spotlessApply. also this is extending the COPIED
// base class sitting in this folder. that copy needs to go (see my note in
// StateMachineSubsystemBase.java) and you import the shared
// frc.robot.subsystems.StateMachineSubsystemBase instead.
public class Elbow extends StateMachineSubsystemBase<ElbowStates> {
  private static Elbow instance;
  // Raymond: make io final, it never gets reassigned.
  private ElbowIO io;
  // Raymond: this is radians (you compare it against MAX_ANGLE which is toRadians). name it
  // targetAngle_rad so nobody feeds it degrees by accident.
  private double targetAngle;
  private ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
  // Raymond: make pid final too.
  private PIDController pid;

  // Raymond: singleton constructor has to be private, otherwise anything can new up a second Elbow.
  // also missing space before the brace - spotlessApply.
  public Elbow(ElbowIO io) {
    super("Elbow");
    this.io = io;
    targetAngle = 0;
    queueState(ElbowStates.IDLE);
    // Raymond: two problems. 1) the kP=1, kI=0 are magic numbers, put PID gains in ElbowConstants
    // like every other subsystem. 2) the third arg is kD, not the loop time - you're passing
    // CHANGE_IN_TIME (0.02) as the derivative gain. that's a bug. set a real kD constant.
    pid = new PIDController(1, 0, ElbowConstants.CHANGE_IN_TIME);
  }

  public static Elbow getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Elbow(new ElbowSim());
          break;
        case REAL:
          instance = new Elbow(new ElbowReal());
          break;
        default:
          instance = new Elbow(new ElbowIO() {});
          break;
      }
    }
    return instance;
  }

  // Raymond: where's inputPeriodic? you never override it, so io.updateInputs(inputs) and
  // Logger.processInputs never run. that means inputs is always default zeros AND in sim the joint
  // never gets ticked - the whole subsystem does nothing and nothing logs. add it like Servo does:
  // @Override public void inputPeriodic() { io.updateInputs(inputs); Logger.processInputs("Elbow",
  // inputs); }
  @Override
  public void handleStateMachine() {
    // Raymond: no default case. if you ever add a state and forget to handle it this silently does
    // nothing - add a default that stops the motor for safety. there's also no DISABLED state at
    // all (see ElbowStates).
    switch (getState()) {
      case INCREASING_ELEVATION_ANGLE:
        if (inputs.atMaxAngle) {
          io.stopMotor();
          queueState(ElbowStates.IDLE);
          break;
        }
        swivelAngle();
        // Raymond: 1.0 is a magic tolerance - make it ElbowConstants.TOLERANCE_RAD. and this whole
        // "reached?" check is duplicated in 4 places, pull it into a isValueReached(tol) helper
        // like Servo/Climber do so it's one Math.abs(pos - target) < tol.
        if (inputs.elbowRotateAngle >= targetAngle - 1.0) {
          queueState(ElbowStates.IDLE);
        }
        break;
      case DECREASING_ELEVATION_ANGLE:
        if (inputs.atMinAngle) {
          io.stopMotor();
          queueState(ElbowStates.IDLE);
          break;
        }
        swivelAngle();
        // Raymond: same magic 1.0 tolerance, use the constant.
        if (inputs.elbowRotateAngle <= targetAngle + 1.0) {
          queueState(ElbowStates.IDLE);
        }
        break;
      case IDLE:
        swivelAngle();
        break;
    }
  }

  public void setTargetAngle(double angle) {

    targetAngle = angle;
    double error = targetAngle - inputs.elbowRotateAngle;

    // Raymond: 1.0 / -1.0 magic deadband again - same TOLERANCE_RAD constant.
    if (error > 1.0) {
      queueState(ElbowStates.INCREASING_ELEVATION_ANGLE);
    } else if (error < -1.0) {
      queueState(ElbowStates.DECREASING_ELEVATION_ANGLE);
    } else {
      queueState(ElbowStates.IDLE);
    }
  }

  @Override
  protected void outputPeriodic() {
    Logger.recordOutput("Elbow/State", getState());
    Logger.recordOutput("Elbow/TargetAngle", targetAngle);
  }

  public void swivelAngle() {
    double currentAngle = inputs.elbowRotateAngle;
    double volts = pid.calculate(currentAngle, targetAngle);
    // Raymond: two things. the output isn't clamped - clamp to +/-12V (constant) before it hits the
    // motor. and a flat GRAVITY_FF is wrong for an arm: gravity load changes with angle, so scale
    // it by Math.cos(currentAngle) or it'll fight you at the extremes.
    io.setElbowVoltage(volts + ElbowConstants.GRAVITY_FF);
  }
}
