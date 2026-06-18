package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;


public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    private PIDController pid;
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private double targetPosition = 0;
    private static Elevator instance;

    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        queueState(ElevatorStates.MOVING_UP);
        pid = new PIDController(1.0, 0.0, 0.2);
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
                if(inputs.atBottom){
                    io.stopMoving();
                }
                else{
                    moveElevator();
                }
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

}