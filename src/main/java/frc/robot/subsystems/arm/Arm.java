package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

public class Arm extends StateMachineSubsystemBase<ArmStates> {
    private static Arm instance;
    private final ArmIO io;

    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    private double shoulderTarget_RAD;
    private double elbowTarget_RAD;

    public Arm(ArmIO io) {
        super("Arm");
        this.io = io;

        shoulderTarget_RAD = 0;
        elbowTarget_RAD = 0;

        queueState(ArmStates.IDLE);
    }

    public static Arm getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Arm(new ArmIOSim());
                    break;
                case REAL:
                    instance = new Arm(new ArmIOReal());
                    break;
                default:
                    instance = new Arm(new ArmIO() {});
                    break;
            }
        }
        return instance;
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case TRAVELLING_TO_POSITION:
                if (atLimit()) {
                    queueState(ArmStates.HOLDING_POSITION);
                }

                if (isValueReached(inputs.elbowSwivelAngle_rad, elbowTarget_RAD, ArmConstants.TOLERANCE_RAD) && isValueReached(inputs.shoulderSwivelAngle_rad, shoulderTarget_RAD, ArmConstants.TOLERANCE_RAD)) {
                    queueState(ArmStates.HOLDING_POSITION);
                } else { // ethan - in real this will do aboslutely nothing. not good.
                    io.swivelElbow();
                    io.swivelShoulder();
                }
                break;

            case HOLDING_POSITION:
                if (!isValueReached(inputs.elbowSwivelAngle_rad, elbowTarget_RAD, ArmConstants.TOLERANCE_RAD) || !isValueReached(inputs.shoulderSwivelAngle_rad, shoulderTarget_RAD, ArmConstants.TOLERANCE_RAD)) {
                    queueState(ArmStates.TRAVELLING_TO_POSITION);
                } else { // ethan - in real this will do absolutely nothing. not good.
                    io.swivelElbow();
                    io.swivelShoulder();
                }
                break;

            case IDLE:
                io.stopMotor();
                break;
            case DISABLED:
                io.stopMotor();
                break;

            default:
                io.stopMotor();
                break;
        }
    }

    public void requestState(ArmStates state) {
        queueState(state);
    }

    /* ethan - honestly break this into 2 methods, one for elbow one for shoulder. */
    public void setArmTargetAngle(double shoulderAngle, double elbowAngle, boolean isAngleInDegrees) {
        
        if (isAngleInDegrees) {
            shoulderAngle = Math.toRadians(shoulderAngle);
            elbowAngle = Math.toRadians(elbowAngle);
        }

        shoulderTarget_RAD = MathUtil.clamp( shoulderAngle, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE);
        elbowTarget_RAD = MathUtil.clamp(elbowAngle, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE);
        
        io.setShoulderTargetAngle(shoulderTarget_RAD);
        io.setElbowTargetAngle(elbowTarget_RAD);


        if (!isValueReached(inputs.elbowSwivelAngle_rad, elbowTarget_RAD, ArmConstants.TOLERANCE_RAD) || !isValueReached(inputs.shoulderSwivelAngle_rad, shoulderTarget_RAD, ArmConstants.TOLERANCE_RAD)) {
            queueState(ArmStates.TRAVELLING_TO_POSITION);
        } else {
            queueState(ArmStates.HOLDING_POSITION);
        }
    }
    
    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Arm/State", getState());
        Logger.recordOutput("Arm/ShoulderTarget", shoulderTarget_RAD);
        Logger.recordOutput("Arm/ElbowTarget", elbowTarget_RAD);
        Logger.recordOutput("Arm/ShoulderVoltage", inputs.shoulderVoltage_volts);
        Logger.recordOutput("Arm/ElbowVoltage", inputs.elbowVoltage_volts);
    }

    public boolean atLimit() {
        return (inputs.shoulderSwivelAngle_rad >= ArmConstants.SHOULDER_MAX_ANGLE
                || inputs.shoulderSwivelAngle_rad <= ArmConstants.SHOULDER_MIN_ANGLE)
                || (inputs.elbowSwivelAngle_rad >= ArmConstants.ELBOW_MAX_ANGLE
                || inputs.elbowSwivelAngle_rad <= ArmConstants.ELBOW_MIN_ANGLE);
    }

    public boolean isValueReached(double position_rad, double target_rad, double tolerance_rad) {
        return Math.abs(position_rad - target_rad) <= tolerance_rad;
    }

    public boolean isAtTargetPosition() {
        return isValueReached(inputs.elbowSwivelAngle_rad, elbowTarget_RAD, ArmConstants.TOLERANCE_RAD) && isValueReached(inputs.shoulderSwivelAngle_rad, shoulderTarget_RAD, ArmConstants.TOLERANCE_RAD);
    }
}