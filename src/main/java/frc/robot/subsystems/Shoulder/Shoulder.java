package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;


public class Shoulder extends StateMachineSubsystemBase<ShoulderStates>{
    private static Shoulder instance;
    private final ShoulderIO io;
    private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private PIDController pid;
    private double targetAngle_rad;

    public Shoulder(ShoulderIO io){
        super("Shoulder");
        this.io = io;
        queueState(ShoulderStates.IDLE);
        pid = new PIDController(ShoulderConstants.KP, ShoulderConstants.KI, ShoulderConstants.KD);
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
                if (inputs.atMaxAngleRad) {
                    io.stopMotor();
                    queueState(ShoulderStates.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.shoulderSwivelAngle_rad >= targetAngle_rad - ShoulderConstants.TOLERANCE_RAD) {
                    queueState(ShoulderStates.IDLE);
                }
                break;

            case DECREASE_SHOOTING_ANGLE:

                if (inputs.atMinAngleRad) {
                    io.stopMotor();
                    queueState(ShoulderStates.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.shoulderSwivelAngle_rad <= targetAngle_rad + ShoulderConstants.TOLERANCE_RAD) {
                    queueState(ShoulderStates.IDLE);
                }
                break;

            case IDLE:
                swivelAngle();
                break;
            case DISABLED:
                io.stopMotor();
                break;
        }
    }

    @Override
    public void inputPeriodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Shoulder", inputs);
    }

    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Shoulder/State", getState());
        Logger.recordOutput("Shoulder/TargetAngle", targetAngle_rad);
    }

    public void requestState(ShoulderStates state) {
        queueState(state);
    }

    public void swivelAngle(){
        double currentAngle = inputs.shoulderSwivelAngle_rad;
        double volts = pid.calculate(currentAngle, targetAngle_rad);
        io.setShoulderVoltage(volts + ShoulderConstants.GRAVITY_FF);
    }

    public void setTargetAngle(double angle) {
        targetAngle_rad = angle;
        double error = targetAngle_rad - inputs.shoulderSwivelAngle_rad;
        if (error > ShoulderConstants.TOLERANCE_RAD) {
            queueState(
                ShoulderStates.INCREASE_SHOOTING_ANGLE);
        }
        else if (error < -ShoulderConstants.TOLERANCE_RAD) {
            queueState(
                ShoulderStates.DECREASE_SHOOTING_ANGLE);
        }
        else {
            queueState(ShoulderStates.IDLE);
        }
    }
}
