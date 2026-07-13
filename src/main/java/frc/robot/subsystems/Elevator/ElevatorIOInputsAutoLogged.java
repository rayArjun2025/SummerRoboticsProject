package frc.robot.subsystems.elevator;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElevatorIOInputsAutoLogged extends ElevatorIO.ElevatorIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ElevatorMotorVolts", elevatorMotorVolts);
    table.put("ElevatorMotorCurrent", elevatorMotorCurrent);
    table.put("ElevatorPositionMeters", elevatorPositionMeters);
    table.put("ElevatorVelocityMetersPerSec", elevatorVelocityMetersPerSec);
    table.put("Connected", connected);
  }

  @Override
  public void fromLog(LogTable table) {
    elevatorMotorVolts = table.get("ElevatorMotorVolts", elevatorMotorVolts);
    elevatorMotorCurrent = table.get("ElevatorMotorCurrent", elevatorMotorCurrent);
    elevatorPositionMeters = table.get("ElevatorPositionMeters", elevatorPositionMeters);
    elevatorVelocityMetersPerSec =
        table.get("ElevatorVelocityMetersPerSec", elevatorVelocityMetersPerSec);
    connected = table.get("Connected", connected);
  }

  @Override
  public ElevatorIOInputsAutoLogged clone() {
    ElevatorIOInputsAutoLogged copy = new ElevatorIOInputsAutoLogged();
    copy.elevatorMotorVolts = this.elevatorMotorVolts;
    copy.elevatorMotorCurrent = this.elevatorMotorCurrent;
    copy.elevatorPositionMeters = this.elevatorPositionMeters;
    copy.elevatorVelocityMetersPerSec = this.elevatorVelocityMetersPerSec;
    copy.connected = this.connected;
    return copy;
  }
}