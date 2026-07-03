package frc.robot.subsystems.elbow;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;


public class Elbow extends StateMachineSubsystemBase<ElbowStates>{
    private static Elbow instance;
    private final ElbowIO io;
    private double targetAngle_RAD;
    private ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
    private final PIDController pid;

    public Elbow(ElbowIO io) {
        super("Elbow");
        this.io = io;
        targetAngle_RAD = 0;
        queueState(ElbowStates.IDLE);
        pid = new PIDController(ElbowConstants.KP, ElbowConstants.KI, ElbowConstants.KD);
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
            case INCREASING_ELEVATION_ANGLE:
                if (inputs.atMaxAngleRad) {
                    io.stopMotor();
                    queueState(ElbowStates.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.elbowRotateAngleRad >= targetAngle_RAD - ElbowConstants.TOLERANCE) {
                    queueState(ElbowStates.IDLE);
                }
                break;
            case DECREASING_ELEVATION_ANGLE:
                if (inputs.atMinAngleRad) {
                    io.stopMotor();
                    queueState(ElbowStates.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.elbowRotateAngleRad <= targetAngle_RAD + ElbowConstants.TOLERANCE) {
                    queueState(ElbowStates.IDLE);
                }
                break;
            case IDLE:
                swivelAngle();
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

        targetAngle_RAD = angle;
        double error = targetAngle_RAD - inputs.elbowRotateAngleRad;

        if (error > ElbowConstants.TOLERANCE) {
            queueState(ElbowStates.INCREASING_ELEVATION_ANGLE);
        }
        else if (error < -ElbowConstants.TOLERANCE) {
            queueState(ElbowStates.DECREASING_ELEVATION_ANGLE);
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
        Logger.processInputs("Elbow", (LoggableInputs)inputs);
    }

    public void swivelAngle(){
        double currentAngle = inputs.elbowRotateAngleRad;
        double volts = pid.calculate(currentAngle, targetAngle_RAD);
        io.setElbowVoltage(volts + ElbowConstants.GRAVITY_FF*Math.cos(currentAngle));
    }
}

