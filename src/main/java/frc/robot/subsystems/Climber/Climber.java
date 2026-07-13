package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

public class Climber extends StateMachineSubsystemBase<ClimberStates> {
    private static Climber instance;

    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

    private double targetDegrees_deg = 0;

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
            case SHALLOW_CLIMB_TRAVELLING:
                /* ethan - leave state switching outside of the switch case. just run io.moveToAngle() */
                if (isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg)) {
                    queueState(ClimberStates.HOLDING);
                }
                else{
                  io.climbTo();
                }
                break;

            case HOLDING:
                /* ethan - leave state switching outside of the switch case. just run io.moveToAngle() */
                if (!isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg)) {
                    queueState(ClimberStates.SHALLOW_CLIMB_TRAVELLING);
                }
                else{
                  io.climbTo();
                }
                break;

            case RELEASING: // ethan - what does this do? could you not just have this as a normal
                            // travelling/holding state with our target angle at homingDegrees_deg?
                setTargetAngle(ClimberConstants.homingDegrees_deg);

                if (isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg)) {
                    queueState(ClimberStates.IDLE);
                }
                else{
                  io.climbTo();
                }
                break;

            case HOMING: // ethan - dupicate of RELEASING state. do we need this?
                setTargetAngle(ClimberConstants.homingDegrees_deg);

                if (isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg)) {
                    io.zeroPosition(); // ethan - you're zeroing?
                    queueState(ClimberStates.IDLE);
                }
                break;

            case IDLE:
                io.stopClimb();
                break;

            case DISABLED:
                io.stopClimb();
                break;

            default:
                io.stopClimb();
                break;
        }
    }

    public void setTargetAngle(double targetDeg) {
        targetDegrees_deg = MathUtil.clamp(targetDeg, ClimberConstants.MIN_DEG, ClimberConstants.MAX_DEG);
        io.setTargetAngle(targetDegrees_deg);

        if (isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg)) {
            queueState(ClimberStates.HOLDING);
        } else {
            queueState(ClimberStates.SHALLOW_CLIMB_TRAVELLING);
        }
    }

    public boolean isValueReached(double position_deg, double target_deg, double tolerance_deg) {
        return Math.abs(position_deg - target_deg) <= tolerance_deg;
    }

    public boolean isHomed() { // ethan - same as isClimbComplete() at homingDegrees_deg. do we
                               // really need this homing/releasing state?
      return isValueReached(inputs.climberPositionDeg, ClimberConstants.homingDegrees_deg, ClimberConstants.tolerance_deg);
    }

    public boolean isClimbComplete() {
      return isValueReached(inputs.climberPositionDeg, targetDegrees_deg, ClimberConstants.tolerance_deg);
    }


    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climber", inputs);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Climber/State", getState());
        Logger.recordOutput("Climber/TargetDegrees", targetDegrees_deg);
    }
}