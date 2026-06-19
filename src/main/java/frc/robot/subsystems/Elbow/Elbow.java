package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.Logger;

public class Elbow extends StateMachineSubsystemBase<ElbowStates>{
    private ElbowIO io;
    private double targetAngle;

    public Elbow(ElbowIO io){
        super("Elbow");
        this.io = io;
        targetAngle = 0;
        queueState(ElbowStates.IDLE);
    }
    
    public void handleStateMachine() {
        
            
    }

    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Elbow/State", getState());
        Logger.recordOutput("Elbow/TargetAngle", targetAngle);
    }
}

