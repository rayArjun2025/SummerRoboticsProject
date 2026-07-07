package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;



public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double targetPosition_m = 0;
    private static Elevator instance;

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
    }

    public void requestState(ElevatorStates state) {
        queueState(state);
    }   

    @Override
    public void outputPeriodic() {
        Logger.recordOutput("Elevator/ElevatorState", getState().toString());
        Logger.recordOutput("Elevator/TargetPosition", targetPosition_m);
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
                    instance = new Elevator(new ElevatorIOSim());
                    break;
                case REAL:
                    instance = new Elevator(new ElevatorIOReal());
                    break;
                default:
                    instance = new Elevator(new ElevatorIOSim());
                    break;
            }
        }   
        return instance;
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case TRAVELLING:
                io.moveElevator();
                if (Math.abs(inputs.elevatorPositionMeters - targetPosition_m) < ElevatorConstants.TOLERANCE_METERS) {
                    queueState(ElevatorStates.IDLE);
                }
                break;
            case IDLE:
                io.moveElevator();
                break;
            case DISABLED:
                io.stopMoving();
                break;
            default:
                io.stopMoving();
                break;
        }
    
    }
    
   

    public void setTargetPosition(double position) {
        targetPosition_m = MathUtil.clamp(position, ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT);
        io.setTargetPosition(targetPosition_m);
        if (targetPosition_m > inputs.elevatorPositionMeters) {
            queueState(ElevatorStates.TRAVELLING);
        } else if (targetPosition_m < inputs.elevatorPositionMeters) {
            queueState(ElevatorStates.TRAVELLING);
        }
        else{
            queueState(ElevatorStates.IDLE);
        }
    }

}