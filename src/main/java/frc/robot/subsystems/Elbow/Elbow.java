package frc.robot.subsystems.elbow;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;


public class Elbow extends StateMachineSubsystemBase<ElbowStates>{
    private static Elbow instance;
    private final ElbowIO io;
    private double targetAngle_RAD;
    private ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();

    public Elbow(ElbowIO io) {
        super("Elbow");
        this.io = io;
        targetAngle_RAD = 0;
        queueState(ElbowStates.IDLE);
    }

    public static Elbow getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Elbow(new ElbowIOSim());
                    break;
                case REAL:
                    instance = new Elbow(new ElbowIOReal());
                    break;
                default:
                    instance = new Elbow(new ElbowIO() {});
                    break;
            }
        }
        return instance;
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case MOVING_ELBOW:
                if (atLimit()) {
                    queueState(ElbowStates.IDLE);
                    break;
                }
                io.swivelAngle();
                if (Math.abs(targetAngle_RAD - inputs.elbowRotateAngleRad) <= ElbowConstants.TOLERANCE) {
                    queueState(ElbowStates.IDLE);
                }
                break;
            case IDLE:
                io.swivelAngle();
                break;
            case DISABLED:
                io.stopMotor();
                break;
            default:
                io.stopMotor();
                break;
        }
    }

    public void requestState(ElbowStates state) {
        queueState(state);
    }

    public void setTargetAngle(double angle) {
        targetAngle_RAD = MathUtil.clamp(angle, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE);

        io.setTargetAngle(targetAngle_RAD);
        double error = Math.abs(targetAngle_RAD - inputs.elbowRotateAngleRad);
        if (error > ElbowConstants.TOLERANCE) {
            queueState(ElbowStates.MOVING_ELBOW);
        }
        else {
            queueState(ElbowStates.IDLE);
        }
    }
    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Elbow/State", getState());
        Logger.recordOutput("Elbow/TargetAngle", targetAngle_RAD);
        Logger.recordOutput("Elbow/Voltage", inputs.elbowVoltageVolts);
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elbow", inputs);
    }

    
    public boolean atLimit() {
        return inputs.elbowRotateAngleRad >= ElbowConstants.MAX_ANGLE || inputs.elbowRotateAngleRad <= ElbowConstants.MIN_ANGLE;
    }
}

