package frc.robot.subsystems.shoulder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ShoulderIOInputsAutoLogged extends ShoulderIO.ShoulderIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ShoulderSwivelAngle", shoulderSwivelAngle_rad);
    table.put("ShoulderCurrent", shoulderCurrent_amps);
    table.put("ShoulderVoltage", shoulderVoltage_volts);
    table.put("ShoulderAngularV", angularVelocityRad);

    table.put("atMinAngle", atMinAngleRad);
    table.put("atMaxAngle", atMinAngleRad);
  }

  @Override
  public void fromLog(LogTable table) {

    shoulderVoltage_volts = table.get("ShoulderVoltage", shoulderVoltage_volts);
    shoulderCurrent_amps = table.get("ShoulderCurrent", shoulderCurrent_amps);
    angularVelocityRad = table.get("ShoulderAngularV", angularVelocityRad);
    shoulderCurrent_amps = table.get("ShoulderSwivelAngle", shoulderCurrent_amps);
    atMaxAngleRad = table.get("atMaxAngle", atMaxAngleRad);
    atMinAngleRad = table.get("atMinAngle", atMinAngleRad);
  }

  @Override
  public ShoulderIOInputsAutoLogged clone() {
    ShoulderIOInputsAutoLogged copy =
        new ShoulderIOInputsAutoLogged();

    copy.shoulderVoltage_volts = shoulderVoltage_volts;
    copy.shoulderCurrent_amps = shoulderCurrent_amps;
    copy.angularVelocityRad = angularVelocityRad;

    copy.shoulderSwivelAngle_rad = shoulderSwivelAngle_rad;

    copy.atMaxAngleRad = atMaxAngleRad;
    copy.atMinAngleRad = atMinAngleRad;

    return copy;
  }
}