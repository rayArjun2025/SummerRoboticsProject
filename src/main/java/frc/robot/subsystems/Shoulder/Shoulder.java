// Raymond: capitalized package again - frc.robot.subsystems.shoulder. rename the Shoulder/ folder
// to shoulder/ too. same thing I flagged all over Climber and Hand.
package frc.robot.subsystems.Shoulder;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

// Raymond: no space before the generic/brace and the whole file is loosely formatted - run
// spotlessApply.
// Raymond: this extends the StateMachineSubsystemBase you copied into this folder. delete that copy
// and import frc.robot.util.StateMachineSubsystemBase like Climber and Hand do. the local copy is
// also missing the timer the real base has.
public class Shoulder extends StateMachineSubsystemBase<ShoulderState> {
  private static Shoulder instance;
  // Raymond: make io final, you never reassign it.
  private ShoulderIO io;
  // Raymond: this is right - AutoLogged type, no cast. this is what Climber and Hand should've
  // done.
  private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
  private PIDController pid;
  // Raymond: name it targetAngle_rad - it's radians and the name doesn't say so. also no default
  // means it sits at 0, see the constructor note.
  private double targetAngle;

  // Raymond: singleton constructor has to be private, otherwise anything can new up a second
  // Shoulder.
  public Shoulder(ShoulderIO io) {
    super("Shoulder");
    this.io = io;
    // Raymond: you boot straight into INCREASE_SHOOTING_ANGLE while targetAngle is still 0, so it
    // drives the arm on startup. start in IDLE like Climber/Hand.
    queueState(ShoulderState.INCREASE_SHOOTING_ANGLE);
    // Raymond: real bug - the 3rd PIDController arg is kD, not the loop period. you're handing
    // CHANGE_IN_TIME (0.02) in as the derivative gain. and 1,0 are magic - put kP/kI/kD in
    // ShoulderConstants.
    pid = new PIDController(1, 0, ShoulderConstants.CHANGE_IN_TIME);
  }

  public static Shoulder getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
        case SIM:
          instance = new Shoulder(new ShoulderSim());
          break;
        case REAL:
          instance = new Shoulder(new ShoulderReal());
          break;
        default:
          instance = new Shoulder(new ShoulderIO() {});
          break;
      }
    }
    return instance;
  }

  @Override
  public void handleStateMachine() {
    switch (getState()) {
      case INCREASE_SHOOTING_ANGLE:
        if (inputs.atMaxAngle) {
          io.stopMotor();
          queueState(ShoulderState.IDLE);
          break;
        }
        swivelAngle();
        // Raymond: 1.0 tolerance is a magic number and you repeat it in DECREASE and setTargetAngle
        // - one TOLERANCE_RAD constant. and use Math.abs(angle - target) < TOLERANCE so both
        // directions read the same way.
        if (inputs.shoulderSwivelAngle >= targetAngle - 1.0) {
          queueState(ShoulderState.IDLE);
        }
        break;

      case DECREASE_SHOOTING_ANGLE:
        if (inputs.atMinAngle) {
          io.stopMotor();
          queueState(ShoulderState.IDLE);
          break;
        }
        swivelAngle();
        // Raymond: same 1.0, same constant.
        if (inputs.shoulderSwivelAngle <= targetAngle + 1.0) {
          queueState(ShoulderState.IDLE);
        }
        break;

        // Raymond: IDLE still runs the PID and drives the motor - in Climber/Hand IDLE stops it.
        // holding position is fine if that's the intent, but there's no DISABLED state and no
        // default
        // case, so a disabled robot keeps commanding voltage. add a DISABLED that stops the motor.
      case IDLE:
        swivelAngle();
        break;
    }
  }

  @Override
  public void inputPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shoulder", inputs);
  }

  @Override
  protected void outputPeriodic() {
    Logger.recordOutput("Shoulder/State", getState());
    Logger.recordOutput("Shoulder/TargetAngle", targetAngle);
  }

  public void swivelAngle() {
    double currentAngle = inputs.shoulderSwivelAngle;
    double volts = pid.calculate(currentAngle, targetAngle);
    io.setShoulderVoltage(volts + ShoulderConstants.GRAVITY_FF);
  }

  public void setTargetAngle(double angle) {
    targetAngle = angle;
    double error = targetAngle - inputs.shoulderSwivelAngle;
    // Raymond: 1.0 deadband yet again - same TOLERANCE_RAD constant as the cases above.
    if (error > 1.0) {
      queueState(ShoulderState.INCREASE_SHOOTING_ANGLE);
    } else if (error < -1.0) {
      queueState(ShoulderState.DECREASE_SHOOTING_ANGLE);
    } else {
      queueState(ShoulderState.IDLE);
    }
  }
}
