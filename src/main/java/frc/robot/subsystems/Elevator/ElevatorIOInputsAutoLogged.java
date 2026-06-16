package frc.robot.subsystems.Elevator;

import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("elevatorMotor_rps", elevatorMotor_rps);
    table.put("elevatorMotorVolts", elevatorMotorVolts);
    table.put("elevatorMotorCurrent", elevatorMotorCurrent);

    table.put("elevatorPositionMeters", elevatorPositionMeters);
    table.put("elevatorVelocityMetersPerSec", elevatorVelocityMetersPerSec);

    table.put("atTop", atTop);
    table.put("atBottom", atBottom);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorMotor_rps =
        table.get("elevatorMotor_rps", elevatorMotor_rps);

    elevatorMotorVolts =
        table.get("elevatorMotorVolts", elevatorMotorVolts);

    elevatorMotorCurrent =
        table.get("elevatorMotorCurrent", elevatorMotorCurrent);

    elevatorPositionMeters =
        table.get("elevatorPositionMeters", elevatorPositionMeters);

    elevatorVelocityMetersPerSec =
        table.get(
            "elevatorVelocityMetersPerSec",
            elevatorVelocityMetersPerSec);

    atTop = table.get("atTop", atTop);
    atBottom = table.get("atBottom", atBottom);
  }

  @Override
  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy =
        new ElevatorIOInputsAutoLogged();

    copy.elevatorMotor_rps = elevatorMotor_rps;
    copy.elevatorMotorVolts = elevatorMotorVolts;
    copy.elevatorMotorCurrent = elevatorMotorCurrent;

    copy.elevatorPositionMeters = elevatorPositionMeters;
    copy.elevatorVelocityMetersPerSec = elevatorVelocityMetersPerSec;

    copy.atTop = atTop;
    copy.atBottom = atBottom;

    return copy;
  }
}