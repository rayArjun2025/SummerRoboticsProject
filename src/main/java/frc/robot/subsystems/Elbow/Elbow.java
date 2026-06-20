package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;

public class Elbow extends StateMachineSubsystemBase<ElbowStates>{
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
    
    public void handleStateMachine() {
        switch(getState()){
            case INCREASING_ELEVATION_ANGLE:
                if(inputs.atMaxAngle) {
                    queueState(ElbowStates.IDLE);
                }
                else{
                    targetAngle = ElbowConstants.MAX_ANGLE;
                    swivelAngle();
                }
                break;
            case DECREASING_ELEVATION_ANGLE:
                if(inputs.atMinAngle) {
                    queueState(ElbowStates.IDLE);
                }
                else{
                    targetAngle = ElbowConstants.MIN_ANGLE;
                    swivelAngle();
                }
                break;
            case IDLE:
                if(inputs.atMaxAngle){
                    targetAngle = inputs.elbowRotateAngle;
                    swivelAngle();
                }
                else{
                    io.stopMotor();
                }
                break;
        }
            
    }

    @Override
    protected void outputPeriodic(){
        Logger.recordOutput("Elbow/State", getState());
        Logger.recordOutput("Elbow/TargetAngle", targetAngle);
    }

    public void swivelAngle(){
        double currentAngle = inputs.elbowRotateAngle;
        double volts = pid.calculate(currentAngle, targetAngle);
        io.setElbowVoltage(volts + ElbowConstants.GRAVITY_FF);
    }
}

