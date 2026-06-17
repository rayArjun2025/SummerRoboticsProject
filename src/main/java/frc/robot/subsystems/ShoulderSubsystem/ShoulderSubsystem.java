package frc.robot.subsystems.ShoulderSubsystem;

import edu.wpi.first.math.controller.PIDController;

public class ShoulderSubsystem extends StateMachineSubsystemBase<ShoulderState>{
    private ShoulderIO io;
    private ShoulderIOInputsAutoLogged inputs = new ShoulderIOInputsAutoLogged();
    private PIDController pid;

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
                break;
            case DECREASE_SHOOTING_ANGLE:
                if(inputs.atMinAngle) {
                    queueState(ShoulderState.IDLE);
                }
                break;
            case IDLE:
                io.stopMotor();
                break;
        }
    }


    @Override
    protected void outputPeriodic(){
        //Logger.recordOutput("Shoulder", inputs);
    }
    
}
