package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {

    private double motorVoltage = 0;

    private final ElevatorSim elevatorSim;
    private final PIDController elevatorPID;

    private final TrapezoidProfile elevatorProfile;
    private TrapezoidProfile.State elevatorCurrent;
    private TrapezoidProfile.State elevatorGoal;

    private final ElevatorFeedforward elevatorFF;

    public ElevatorIOSim() {
        elevatorProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ElevatorConstants.MAX_VELOCITY,
                ElevatorConstants.MAX_ACCELERATION));

        elevatorCurrent = new TrapezoidProfile.State(
            ElevatorConstants.ELEVATOR_MIN_HEIGHT, 0);

        elevatorGoal = new TrapezoidProfile.State(
            ElevatorConstants.ELEVATOR_MIN_HEIGHT, 0);

        elevatorFF = new ElevatorFeedforward(
            ElevatorConstants.KS,
            ElevatorConstants.KG,
            ElevatorConstants.KV,
            ElevatorConstants.KA);

        elevatorSim = new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT,
            true,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT);

        elevatorPID = new PIDController(
            ElevatorConstants.KP,
            ElevatorConstants.KI,
            ElevatorConstants.KD);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        moveElevator();
        elevatorSim.update(Constants.globalDelta_s);

        inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorMotorVolts = motorVoltage;
        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        inputs.elevatorMotorCurrent = elevatorSim.getCurrentDrawAmps();
        inputs.connected = true;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motorVoltage = voltage;
        elevatorSim.setInputVoltage(motorVoltage);
    }

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }

    @Override
    public void moveElevator() {
        elevatorCurrent = elevatorProfile.calculate(
            Constants.globalDelta_s,
            elevatorCurrent,
            elevatorGoal);

        double pidOut = elevatorPID.calculate(
            elevatorSim.getPositionMeters(),
            elevatorCurrent.position);

        double ff = elevatorFF.calculate(
            elevatorCurrent.position,
            elevatorCurrent.velocity);

        double voltage = MathUtil.clamp(
            pidOut + ff,
            ElevatorConstants.LOW_CLAMP,
            ElevatorConstants.HIGH_CLAMP);

        setMotorVoltage(voltage);
    }

    @Override
    public void setTargetPosition(double targetPosition_m) {
        elevatorGoal = new TrapezoidProfile.State(targetPosition_m, 0);
        elevatorCurrent = new TrapezoidProfile.State( elevatorSim.getPositionMeters(), elevatorSim.getVelocityMetersPerSecond());
    }
}