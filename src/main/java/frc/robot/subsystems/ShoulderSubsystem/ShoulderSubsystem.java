package frc.robot.subsystems.ShoulderSubsystem;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;

public class ShoulderSubsystem extends StateMachineSubsystemBase<ShoulderState>{
    private ShoulderIO io;
    private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private PIDController pid;
    private double targetAngle;

    public ShoulderSubsystem(ShoulderIO io){
        super("Shoulder");
        this.io = io;
        queueState(ShoulderState.IDLE);
        pid = new PIDController(1, 0, 0.02);
    }

    @Override
    public void handleStateMachine() {
        switch(getState()){
            case INCREASE_SHOOTING_ANGLE:
                if(inputs.atMaxAngle) {
                    queueState(ShoulderState.IDLE);
                }
                else{
                    targetAngle = ShoulderConstants.MAX_ANGLE;
                    swivelAngle();
                }
                break;
            case DECREASE_SHOOTING_ANGLE:
                if(inputs.atMinAngle) {
                    queueState(ShoulderState.IDLE);
                }
                else{
                    targetAngle = ShoulderConstants.MIN_ANGLE;
                    swivelAngle();
                }
                break;
            case IDLE:
                targetAngle = inputs.shoulderSwivelAngle;
                swivelAngle();
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
        Logger.recordOutput("Shoulder/TargetAngle", targetAngle);
    }

    public void swivelAngle(){
        double currentAngle = inputs.shoulderSwivelAngle;
        double volts = pid.calculate(currentAngle, targetAngle);
        io.setShoulderVoltage(volts + ShoulderConstants.GRAVITY_FF);
    }

    
}
