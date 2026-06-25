// Raymond: package is capitalized. all our packages are lowercase - frc.robot.subsystems.elevator. rename the whole Elevator/ folder to elevator/ too.
package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;


// Raymond: this extends the COPY of the base sitting in this folder. there's one shared StateMachineSubsystemBase in subsystems/ - use that. delete the local copy (see my note in that file).
public class Elevator extends StateMachineSubsystemBase<ElevatorStates> {
    private final ElevatorIO io;
    // Raymond: pid is never reassigned, make it final.
    private PIDController pid;
    // Raymond: good - this is the AutoLogged type. but make it private final like io.
    ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    // Raymond: name it with units - targetPosition_m. and the 0 is fine as an init but don't leave it as a bare literal if it's meant to be a home height, pull it from ElevatorConstants.
    private double targetPosition = 0;
    private static Elevator instance;

    // Raymond: singleton constructor has to be private. public means anything can new up a second Elevator and bypass getInstance().
    public Elevator(ElevatorIO io) {
        super("Elevator");
        this.io = io;
        // Raymond: you queue MOVING_UP at startup with targetPosition still 0 - so on boot it immediately tries to drive up to 0. start in IDLE.
        queueState(ElevatorStates.MOVING_UP);
        // Raymond: 1.0/0.0/0.2 are magic. PID gains go in ElevatorConstants as kP/kI/kD like the reference climber does.
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
        // Raymond: no default case. if a state ever isn't handled here the elevator just sits with whatever voltage was last set - add a default that stops the motor.
        switch (getState()) {
            case MOVING_UP:
                if (inputs.atTop) {
                    io.stopMoving();
                    queueState(ElevatorStates.IDLE);
                    break;
                }

                moveElevator();
                // Raymond: 0.02 is a magic tolerance, make it a constant like TOLERANCE_METERS in the reference. it also happens to equal CHANGE_IN_TIME in your constants - two unrelated things, don't let them be the same number by accident.
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
                // Raymond: same 0.02 magic tolerance, same constant should cover it.
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
        // Raymond: -12/12 are magic. put the voltage limit in ElevatorConstants. reference does the clamp down in the IO layer (ClimberIOReal.setClimbVoltage) which is the better spot - subsystem shouldn't care about volt limits.
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
        // Raymond: `else{` and the `} else if` above aren't on the same brace style, and the IO files have `implements ElevatorIO{` with no space. just run spotlessApply across the whole folder and stop hand-formatting.
        else{
            queueState(ElevatorStates.IDLE);
        }
    }

}