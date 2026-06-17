package frc.robot.subsystems.ShoulderSubsystem;

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
        pid = new PIDController(1, 0, 0);
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
    protected void outputPeriodic(){
        
    }

    public void swivelAngle(){
        double volts = pid.calculate(inputs.shoulderSwivelAngle, targetAngle);
        io.setShoulderVoltage(volts + ShoulderConstants.GRAVITY_FF);
    }

    
}
