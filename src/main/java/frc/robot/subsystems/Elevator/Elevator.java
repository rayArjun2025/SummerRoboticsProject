package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private final PIDController pid;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double targetPosition = 0;

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
        pid = new PIDController(1.0, 0.0, 0.0);
    }

    @Override
    public void outputPeriodic() {
        Logger.recordOutput("Elevator/ElevatorState", getState().toString());
        Logger.recordOutput("Elevator/TargetPosition", targetPosition);
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    public void handleStateMachine() {
       SmartDashboard.putNumber(
        "Elevator Position",
        inputs.elevatorPositionMeters);

        SmartDashboard.putNumber(
        "Elevator Velocity",
        inputs.elevatorVelocityMetersPerSec);

        SmartDashboard.putNumber(
        "Elevator Voltage",
        inputs.elevatorMotorVolts);

        SmartDashboard.putBoolean(
        "Elevator At Top",
        inputs.atTop);

        SmartDashboard.putBoolean(
        "Elevator At Bottom",
        inputs.atBottom);

        switch (getState()) {

            case MOVING_UP:
                if (inputs.atTop) {
                    queueState(ElevatorStates.IDLE);
                }
                else{
                    targetPosition = ElevatorConstants.ELEVATOR_MAX_HEIGHT;
                    moveElevator();
                }
                break;
            case MOVING_DOWN:
                if (inputs.atBottom) {
                    queueState(ElevatorStates.IDLE);
                }
                else {
                    targetPosition = ElevatorConstants.ELEVATOR_MIN_HEIGHT;
                    moveElevator();
                }
                break;
            case IDLE:
                targetPosition = inputs.elevatorPositionMeters;
                break;
        }
     
    }
    
    public void moveElevator(){
        double currentPosition = inputs.elevatorPositionMeters;
        double pidOut = pid.calculate(currentPosition, targetPosition);
        double ff = ElevatorConstants.GRAVITY_FF;
        double volts = pidOut + ff;
        io.setMotorVoltage(volts);
    }
}