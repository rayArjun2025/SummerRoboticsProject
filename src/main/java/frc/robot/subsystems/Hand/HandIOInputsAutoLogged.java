// Raymond: same as Climber's - this is @AutoLog generated code, and you wrote it but Hand.java
// never uses it (it uses the raw HandIO.HandIOInputs). either delete it and let @AutoLog generate
// it, or actually use this class as the inputs field.
package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HandIOInputsAutoLogged extends HandIO.HandIOInputs implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("HandMotorVolts", handMotorVolts);
    table.put("HandMotorCurrent", handMotorCurrent);
    table.put("HandPositionDeg", handPositionDeg);
    table.put("HandMotorVelocity", handMotorVelocity_dps);
  }

  @Override
  public void fromLog(LogTable table) {
    handMotorVolts = table.get("HandMotorVolts", handMotorVolts);
    handMotorCurrent = table.get("HandMotorCurrent", handMotorCurrent);
    handPositionDeg = table.get("HandPositionDeg", handPositionDeg);
    handMotorVelocity_dps = table.get("HandMotorVelocity", handMotorVelocity_dps);
  }

  @Override
  public HandIOInputsAutoLogged clone() {
    HandIOInputsAutoLogged copy = new HandIOInputsAutoLogged();

    copy.handMotorVolts = handMotorVolts;
    copy.handMotorCurrent = handMotorCurrent;
    copy.handPositionDeg = handPositionDeg;
    copy.handMotorVelocity_dps = handMotorVelocity_dps;

    return copy;
  }
}
