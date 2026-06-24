package frc.robot.subsystems.ShoulderSubsystem;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShoulderIOInputsAutoLogged extends ShoulderIO.ShoulderIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ShoulderSwivelAngle", shoulderSwivelAngle);
    table.put("ShoulderCurrent", shoulderCurrent);
    table.put("ShoulderVoltage", shoulderVoltage);
    table.put("ShoulderAngularV", angularVelocityRad);

    table.put("atMinAngle", atMinAngle);
    table.put("atMaxAngle", atMaxAngle);
  }

  @Override
  public void fromLog(LogTable table) {

    shoulderVoltage = table.get("ShoulderVoltage", shoulderVoltage);
    shoulderCurrent = table.get("ShoulderCurrent", shoulderCurrent);
    angularVelocityRad = table.get("ShoulderAngularV", angularVelocityRad);
    shoulderSwivelAngle = table.get("ShoulderSwivelAngle", shoulderSwivelAngle);
    atMaxAngle = table.get("atMaxAngle", atMaxAngle);
    atMinAngle = table.get("atMinAngle", atMinAngle);
  }

  @Override
  public ShoulderIOInputsAutoLogged clone() {
    ShoulderIOInputsAutoLogged copy =
        new ShoulderIOInputsAutoLogged();

    copy.shoulderVoltage = shoulderVoltage;
    copy.shoulderCurrent = shoulderCurrent;
    copy.angularVelocityRad = angularVelocityRad;

    copy.shoulderSwivelAngle = shoulderSwivelAngle;

    copy.atMaxAngle = atMaxAngle;
    copy.atMinAngle = atMinAngle;

    return copy;
  }
}