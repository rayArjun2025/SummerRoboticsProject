package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;


public class Shoulder extends StateMachineSubsystemBase<ShoulderStates>{
    private static Shoulder instance;
    private final ShoulderIO io;
    private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private double targetAngle_rad;

    public Shoulder(ShoulderIO io){
        super("Shoulder");
        this.io = io;
        targetAngle_rad = inputs.shoulderSwivelAngle_rad;   
        queueState(ShoulderStates.IDLE);
        
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
            case TRAVELLING_TO_POSITION:
                io.setTargetAngle(targetAngle_rad);
                if(atTarget()) {
                    queueState(ShoulderStates.HOLDING_POSITION);
                }
                break;

            case HOLDING_POSITION:
                io.setTargetAngle(targetAngle_rad);
                break;
            
            case IDLE:
                io.setTargetAngle(targetAngle_rad);
                break;

            case DISABLED:
                io.stopMotor();
                break;

            default:
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

    public void queueTargetAngle(double angle) {
        angle = MathUtil.clamp(angle, ShoulderConstants.MIN_ANGLE, ShoulderConstants.MAX_ANGLE);
        
        targetAngle_rad = angle;
        double error = targetAngle_rad - inputs.shoulderSwivelAngle_rad;
        if (error > ShoulderConstants.TOLERANCE_RAD) {
            queueState(ShoulderStates.TRAVELLING_TO_POSITION);
        }
        else if (error < -ShoulderConstants.TOLERANCE_RAD) {
            queueState(ShoulderStates.TRAVELLING_TO_POSITION);
        }
        else {
            queueState(ShoulderStates.HOLDING_POSITION);
        }
    }

    private boolean atTarget() {
        return Math.abs(targetAngle_rad - inputs.shoulderSwivelAngle_rad)  <= ShoulderConstants.TOLERANCE_RAD;
    }
}
