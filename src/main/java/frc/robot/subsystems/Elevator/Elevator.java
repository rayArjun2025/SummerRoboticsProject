package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;



public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private final PIDController pid;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double targetPosition_m = 0;
    private static Elevator instance;

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
        pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
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
            case MOVING_UP:
                if (inputs.atTop) {
                    io.stopMoving();
                    queueState(ElevatorStates.IDLE);
                    break;
                }

                moveElevator();
                if (inputs.elevatorPositionMeters >= targetPosition_m - ElevatorConstants.TOLERANCE_METERS) {
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
                if (inputs.elevatorPositionMeters <= targetPosition_m + ElevatorConstants.TOLERANCE_METERS) {
                    queueState(ElevatorStates.IDLE);
                }
                break;
            case IDLE:
                moveElevator();
                break;
            default:
                io.stopMoving();
                break;
        }
    
    }
    
    public void moveElevator(){
        double currentPosition = inputs.elevatorPositionMeters;
        double ff = ElevatorConstants.GRAVITY_FF;
        double pidOut = pid.calculate(currentPosition, targetPosition_m);
        double volts = MathUtil.clamp(pidOut + ff, ElevatorConstants.LOW_CLAMP, ElevatorConstants.HIGH_CLAMP);
        io.setMotorVoltage(volts);
    }

    public void setTargetPosition(double position) {
        targetPosition_m = position;

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