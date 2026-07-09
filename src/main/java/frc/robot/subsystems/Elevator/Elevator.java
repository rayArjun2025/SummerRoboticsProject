package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;
import frc.robot.util.StateMachineSubsystemBase;

public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private static Elevator instance;

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private double targetPosition_m = 0;

    private Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.IDLE);
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

    public void requestState(ElevatorStates state) {
        queueState(state);
    }

    @Override
    public void handleStateMachine() {
        switch (getState()) {
            case TRAVELLING:

                if (atLimit()) {
                    queueState(ElevatorStates.HOLDING);
                }

                if (isAtTargetPosition()) {
                    queueState(ElevatorStates.HOLDING);
                }
                else{
                    io.moveElevator();
                }
                break;

            case HOLDING:
                if (!isAtTargetPosition()) {
                    queueState(ElevatorStates.TRAVELLING);
                }
                else{
                    io.moveElevator();
                }
                break;

            case IDLE:
                io.stopMoving();
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
        targetPosition_m = MathUtil.clamp(
            position,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT);

        io.setTargetPosition(targetPosition_m);

        if (isAtTargetPosition()) {
            queueState(ElevatorStates.HOLDING);
        } else {
            queueState(ElevatorStates.TRAVELLING);
        }
    }

    public boolean isAtTargetPosition() {
        return Math.abs(inputs.elevatorPositionMeters - targetPosition_m) < ElevatorConstants.TOLERANCE_METERS;
    }

    public boolean atLimit() {
        return inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT
            || inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    }

    @Override
    public void inputPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Elevator/State", getState());
        Logger.recordOutput("Elevator/TargetPosition", targetPosition_m);
    }
}