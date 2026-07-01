package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;



public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private PIDController pid;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double targetPosition = 0;
    private static Elevator instance;

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
        pid = new PIDController(1.0, 0.0, 0.2);
    }

    public void requestState(ElevatorStates state) {
        queueState(state);
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

    public static Elevator getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
                case SIM:
                    instance = new Elevator(new ElevatorSimulation());
                    break;
                case REAL:
                    instance = new Elevator(new ElevatorReal());
                    break;
                default:
                    instance = new Elevator(new ElevatorSimulation());
                    break;
            }
        }   
        return instance;
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case MOVING_UP:
                if (inputs.atTop) {
                    io.stopMoving();
                    queueState(ElevatorStates.IDLE);
                    break;
                }

                moveElevator();
                if (inputs.elevatorPositionMeters >= targetPosition - 0.02) {
                    queueState(ElevatorStates.IDLE);
                }
                break;
            case MOVING_DOWN:
                if (inputs.atBottom) {
                    io.stopMoving();
                    queueState(ElevatorStates.IDLE);
                    break;
                }
                moveElevator();
                if (inputs.elevatorPositionMeters <= targetPosition + 0.02) {
                    queueState(ElevatorStates.IDLE);
                }
                break;
            case IDLE:
                moveElevator();
                break;
        }
    
    }
    
    public void moveElevator(){
        double currentPosition = inputs.elevatorPositionMeters;
        double ff = ElevatorConstants.GRAVITY_FF;
        double pidOut = pid.calculate(currentPosition, targetPosition);
        double volts = MathUtil.clamp(pidOut + ff, -12, 12);
        io.setMotorVoltage(volts);
    }

    public void setTargetPosition(double position) {
        targetPosition = position;

        if (position > inputs.elevatorPositionMeters) {
            queueState(ElevatorStates.MOVING_UP);
        } else if (position < inputs.elevatorPositionMeters) {
            queueState(ElevatorStates.MOVING_DOWN);
        }
        else{
            queueState(ElevatorStates.IDLE);
        }
    }

}