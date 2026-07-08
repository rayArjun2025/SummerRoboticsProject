package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;


public class Arm extends StateMachineSubsystemBase<ArmStates>{
    private static Arm instance;
    private final ArmIO io;
    private double shoulderTarget_RAD, elbowTarget_RAD;

    private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

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
                }

                else{
                    io.swivelElbow();
                    io.swivelShoulder();
                }

                break;
                
            case HOLDING_POSITION:
                if (!isValueReached(inputs.elbowSwivelAngle_rad, elbowTarget_RAD, ArmConstants.TOLERANCE_RAD) || !isValueReached(inputs.shoulderSwivelAngle_rad, shoulderTarget_RAD, ArmConstants.TOLERANCE_RAD)) {
                    queueState(ArmStates.TRAVELLING_TO_POSITION);
                }
                else{
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

    public void setShoulderTargetAngle(double angle) {
        shoulderTarget_RAD = MathUtil.clamp(angle, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE);
        io.setShoulderTargetAngle(shoulderTarget_RAD);

        double error = Math.abs(shoulderTarget_RAD - inputs.shoulderSwivelAngle_rad);
        if (error > ArmConstants.TOLERANCE_RAD) {
            queueState(ArmStates.TRAVELLING_TO_POSITION);
        }
        else {
            queueState(ArmStates.HOLDING_POSITION);
        }
    }

    public void setElbowTargetAngle(double angle) {
        elbowTarget_RAD = MathUtil.clamp(angle, ArmConstants.ELBOW_MIN_ANGLE, ArmConstants.ELBOW_MAX_ANGLE);
        io.setElbowTargetAngle(elbowTarget_RAD);

        double error = Math.abs(elbowTarget_RAD - inputs.elbowSwivelAngle_rad);
        if (error > ArmConstants.TOLERANCE_RAD) {
            queueState(ArmStates.TRAVELLING_TO_POSITION);
        }
        else {
            queueState(ArmStates.HOLDING_POSITION);
        }
    }
    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Arm/State", getState());
        Logger.recordOutput("Arm/TargetAngle", elbowTarget_RAD);
        Logger.recordOutput("Arm/Voltage", inputs.elbowVoltage_volts);
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);
    }

    
    public boolean atLimit() {
        return (inputs.shoulderSwivelAngle_rad >= ArmConstants.SHOULDER_MAX_ANGLE || inputs.shoulderSwivelAngle_rad <= ArmConstants.SHOULDER_MIN_ANGLE) && (inputs.elbowSwivelAngle_rad >= ArmConstants.ELBOW_MAX_ANGLE || inputs.elbowSwivelAngle_rad <= ArmConstants.ELBOW_MIN_ANGLE);
    }

    public boolean isValueReached(double position_deg, double target_deg, double tolerance_deg) {
        return (Math.abs(position_deg - target_deg) <= tolerance_deg);
    }
}

