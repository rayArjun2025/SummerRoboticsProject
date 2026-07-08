package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

// Raymond: name the file/class ElevatorIOReal to match the IO naming (ClimberIOReal, etc).
// ElevatorReal reads like it's not an IO impl.
public class ElevatorReal implements ElevatorIO {
  private final TalonFX elevatorMotor;
  // Raymond: these signals should be private. nothing outside this class touches them.
  public final StatusSignal<Current> elevatorCurrent;
  public final StatusSignal<Voltage> elevatorVoltage;
  public final StatusSignal<AngularVelocity> elevatorVelocity;
  public final StatusSignal<Angle> elevatorPosition;

  public ElevatorReal() {
    elevatorMotor = new TalonFX(ElevatorConstants.MOTOR_ID, TunerConstants.kCANBus);
    elevatorCurrent = elevatorMotor.getStatorCurrent();
    elevatorVoltage = elevatorMotor.getMotorVoltage();
    elevatorVelocity = elevatorMotor.getVelocity();
    elevatorPosition = elevatorMotor.getPosition();

    // Raymond: you never configure the motor. no TalonFXConfiguration, no current limit (we run
    // stator ~60/supply ~80), no gear ratio, no inverted/neutral mode. look at ClimberIOReal - you
    // need a config block here or this thing will brown out / coast when disabled.
    // Raymond: 50 is a magic number - it's the update rate, pull it from a constant (reference uses
    // Constants.globalDelta_Hz).
    BaseStatusSignal.setUpdateFrequencyForAll(
        50, elevatorCurrent, elevatorVoltage, elevatorVelocity, elevatorPosition);
    elevatorMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // Raymond: you refreshAll twice in a row - the first call does nothing useful, just delete it
    // and keep the one that sets connected.
    BaseStatusSignal.refreshAll(
        elevatorCurrent, elevatorVoltage, elevatorVelocity, elevatorPosition);
    inputs.connected =
        BaseStatusSignal.refreshAll(
                elevatorCurrent, elevatorVoltage, elevatorVelocity, elevatorPosition)
            .isOK();
    inputs.elevatorMotorCurrent = elevatorCurrent.getValueAsDouble();
    inputs.elevatorMotorVolts = elevatorVoltage.getValueAsDouble();
    inputs.elevatorVelocityMetersPerSec = convertToLinearVel(elevatorVelocity.getValueAsDouble());
    inputs.elevatorPositionMeters = convertToMeters(elevatorPosition.getValueAsDouble());
    // Raymond: 0.001 magic tolerance, twice. make it a constant in ElevatorConstants.
    inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT - 0.001;
    inputs.atBottom =
        inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT + 0.001;
  }

  private double convertToLinearVel(double aVelocity) {
    return aVelocity / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
  }

  private double convertToMeters(double aPos) {
    return aPos / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
  }

  @Override
  public void setMotorVoltage(double voltage) {
    elevatorMotor.setVoltage(voltage);
  }

  @Override
  public void stopMoving() {
    setMotorVoltage(0);
  }
}
