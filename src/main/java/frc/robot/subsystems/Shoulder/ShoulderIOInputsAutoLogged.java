// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
// Raymond: delete this whole file. you put @AutoLog on ShoulderIOInputs, which already generates a
// ShoulderIOInputsAutoLogged in this exact package - this hand-written one collides with the
// generated class. and your toLog/fromLog/clone here forget the `connected` field anyway, so it
// wouldn't even log correctly. let @AutoLog do it.
package frc.robot.subsystems.Shoulder;

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
    ShoulderIOInputsAutoLogged copy = new ShoulderIOInputsAutoLogged();

    copy.shoulderVoltage = shoulderVoltage;
    copy.shoulderCurrent = shoulderCurrent;
    copy.angularVelocityRad = angularVelocityRad;

    copy.shoulderSwivelAngle = shoulderSwivelAngle;

    copy.atMaxAngle = atMaxAngle;
    copy.atMinAngle = atMinAngle;

    return copy;
  }
}
