package frc.robot.subsystems.Hand;

import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class HandIOInputsAutoLogged extends HandIO.HandIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("handCurrent", handMotorCurrent);
    table.put("handVoltage", handMotorVolts);

    table.put("fullOpen", fullOpen);
    table.put("fullClose", fullClose);
  }

  @Override
  public void fromLog(LogTable table) {
    handMotorCurrent = table.get("handCurrent", handMotorCurrent);
    handMotorVolts = table.get("handVoltage", handMotorVolts);

    table.get("fullOpen", fullOpen);
    table.get("fullClose", fullClose);
  }

  @Override
  public HandIOInputsAutoLogged clone() {
    HandIOInputsAutoLogged copy =
        new HandIOInputsAutoLogged();

    copy.handMotorCurrent = handMotorCurrent;
    copy.handMotorVolts = handMotorVolts;

    copy.fullOpen = fullOpen;
    copy.fullClose = fullClose;

    return copy;
  }
}