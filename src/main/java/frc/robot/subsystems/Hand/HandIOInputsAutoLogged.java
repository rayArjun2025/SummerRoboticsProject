package frc.robot.subsystems.hand;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HandIOInputsAutoLogged extends HandIO.HandIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("Connected", connected);
    table.put("HandMotorVolts", handMotorVolts);
    table.put("HandMotorCurrent", handMotorCurrent);
    table.put("HandPositionDeg", handPositionDeg);
    table.put("HandMotorVelocity_dps", handMotorVelocity_dps);
  }

  @Override
  public void fromLog(LogTable table) {
    connected = table.get("Connected", connected);
    handMotorVolts = table.get("HandMotorVolts", handMotorVolts);
    handMotorCurrent = table.get("HandMotorCurrent", handMotorCurrent);
    handPositionDeg = table.get("HandPositionDeg", handPositionDeg);
    handMotorVelocity_dps = table.get("HandMotorVelocity_dps", handMotorVelocity_dps);
  }

  @Override
  public HandIOInputsAutoLogged clone() {
    HandIOInputsAutoLogged copy = new HandIOInputsAutoLogged();
    copy.connected = this.connected;
    copy.handMotorVolts = this.handMotorVolts;
    copy.handMotorCurrent = this.handMotorCurrent;
    copy.handPositionDeg = this.handPositionDeg;
    copy.handMotorVelocity_dps = this.handMotorVelocity_dps;
    return copy;
  }
}