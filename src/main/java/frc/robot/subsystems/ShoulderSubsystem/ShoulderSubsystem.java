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
        queueState(ShoulderState.INCREASE_SHOOTING_ANGLE);
        pid = new PIDController(1, 0, ShoulderConstants.CHANGE_IN_TIME);
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case INCREASE_SHOOTING_ANGLE:
                if (inputs.atMaxAngle) {
                    io.stopMotor();
                    queueState(ShoulderState.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.shoulderSwivelAngle >= targetAngle - 1.0) {
                    queueState(ShoulderState.IDLE);
                }
                break;

            case DECREASE_SHOOTING_ANGLE:

                if (inputs.atMinAngle) {
                    io.stopMotor();
                    queueState(ShoulderState.IDLE);
                    break;
                }
                swivelAngle();
                if (inputs.shoulderSwivelAngle <= targetAngle + 1.0) {
                    queueState(ShoulderState.IDLE);
                }
                break;

            case IDLE:
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

    public void setTargetAngle(double angle) {
        targetAngle = angle;
        double error = targetAngle - inputs.shoulderSwivelAngle;
        if (error > 1.0) {
            queueState(
                ShoulderState.INCREASE_SHOOTING_ANGLE);
        }
        else if (error < -1.0) {
            queueState(
                ShoulderState.DECREASE_SHOOTING_ANGLE);
        }
        else {
            queueState(ShoulderState.IDLE);
        }
    }
}
