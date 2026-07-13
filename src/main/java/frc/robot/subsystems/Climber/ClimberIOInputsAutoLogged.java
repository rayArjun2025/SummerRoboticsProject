package frc.robot.subsystems.climber;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ClimberIOInputsAutoLogged extends ClimberIO.ClimberIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ClimberCurrent", climberCurrent);
    table.put("ClimberVoltage", climberVoltage);
    table.put("ClimberPositionDeg", climberPositionDeg);
    table.put("ClimberVelocity_dps", climberVelocity_dps);
    table.put("Connected", connected);
  }

  @Override
  public void fromLog(LogTable table) {
    climberCurrent = table.get("ClimberCurrent", climberCurrent);
    climberVoltage = table.get("ClimberVoltage", climberVoltage);
    climberPositionDeg = table.get("ClimberPositionDeg", climberPositionDeg);
    climberVelocity_dps = table.get("ClimberVelocity_dps", climberVelocity_dps);
    connected = table.get("Connected", connected);
  }

  @Override
  public ClimberIOInputsAutoLogged clone() {
    ClimberIOInputsAutoLogged copy = new ClimberIOInputsAutoLogged();
    copy.climberCurrent = this.climberCurrent;
    copy.climberVoltage = this.climberVoltage;
    copy.climberPositionDeg = this.climberPositionDeg;
    copy.climberVelocity_dps = this.climberVelocity_dps;
    copy.connected = this.connected;
    return copy;
  }
}