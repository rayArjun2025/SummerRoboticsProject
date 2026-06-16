package frc.robot.subsystems.ShoulderSubsystem;


public class ShoulderSubsystem extends StateMachineSubsystemBase<ShoulderState>{
    private ShoulderIO io;

    public ShoulderSubsystem(ShoulderIO io){
        super("Shoulder");
        this.io = io;
    }

    @Override
    public void handleStateMachine() {
        switch(getState()){

            case INCREASE_SHOOTING_ANGLE:
                break;
            case DECREASE_SHOOTING_ANGLE:
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
