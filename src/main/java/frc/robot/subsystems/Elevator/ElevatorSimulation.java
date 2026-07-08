package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

// Raymond: rename to ElevatorIOSim to match the IO naming. and good - this is an actual
// ElevatorSim, position/velocity move, so the state machine can reach targets in sim. that's
// exactly what we want.
public class ElevatorSimulation implements ElevatorIO {
  private double motorVoltage = 0;
  private final ElevatorSim elevatorSim;

  public ElevatorSimulation() {

    elevatorSim =
        new ElevatorSim(
            DCMotor.getKrakenX60Foc(1),
            ElevatorConstants.GEAR_RATIO,
            ElevatorConstants.CARRIAGE_MASS,
            ElevatorConstants.DRUM_RADIUS,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT,
            ElevatorConstants.ELEVATOR_MAX_HEIGHT,
            true,
            ElevatorConstants.ELEVATOR_MIN_HEIGHT);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {

    elevatorSim.update(ElevatorConstants.CHANGE_IN_TIME);

    inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
    inputs.elevatorMotorVolts = motorVoltage;
    inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
    inputs.elevatorMotorCurrent = elevatorSim.getCurrentDrawAmps();

    // Raymond: same 0.001 magic tolerance as the real IO - move it to a shared constant so sim and
    // real stay in sync.
    inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT - 0.001;
    inputs.atBottom =
        inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT + 0.001;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    motorVoltage = voltage;
    elevatorSim.setInputVoltage(voltage);
  }

  @Override
  public void stopMoving() {
    setMotorVoltage(0);
  }
}
