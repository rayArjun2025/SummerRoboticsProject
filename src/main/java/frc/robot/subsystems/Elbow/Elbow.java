package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;


public class Elbow extends StateMachineSubsystemBase<ElbowStates>{
    private static Elbow instance;
    private ElbowIO io;
    private double targetAngle;
    private ElbowIOInputsAutoLogged inputs = new ElbowIOInputsAutoLogged();
    private PIDController pid;

    public Elbow(ElbowIO io){
        super("Elbow");
        this.io = io;
        targetAngle = 0;
        queueState(ElbowStates.IDLE);
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

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case INCREASING_ELEVATION_ANGLE:
                if (inputs.atMaxAngle) {
                    io.stopMotor();
                    queueState(ElbowStates.IDLE);
                    break;
                }
                swivelAngle();
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
                if (inputs.elbowRotateAngle <= targetAngle + 1.0) {
                    queueState(ElbowStates.IDLE);
                }
                break;
            case IDLE:
                swivelAngle();
                break;
        }
    }

    public void requestState(ElbowStates state) {
        queueState(state);
    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;
        double error = targetAngle - inputs.elbowRotateAngle;

        if (error > 1.0) {
            queueState(ElbowStates.INCREASING_ELEVATION_ANGLE);
        }
        else if (error < -1.0) {
            queueState(ElbowStates.DECREASING_ELEVATION_ANGLE);
        }
        else {
            queueState(ElbowStates.IDLE);
        }
    }
    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Elbow/State", getState());
        Logger.recordOutput("Elbow/TargetAngle", targetAngle);
        Logger.recordOutput("Elbow/Voltage", inputs.elbowVoltage);
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elbow", inputs);
    }

    public void swivelAngle(){
        double currentAngle = inputs.elbowRotateAngle;
        double volts = pid.calculate(currentAngle, targetAngle);
        io.setElbowVoltage(volts + ElbowConstants.GRAVITY_FF);
    }
}

