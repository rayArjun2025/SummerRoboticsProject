// Raymond: delete this whole file. you already put @AutoLog on ElbowIOInputs, which GENERATES a class with this exact name - this hand-written copy collides with the generated one. let AutoLog do it, that's the whole point. on top of that this copy silently drops the `connected` field from toLog/fromLog/clone, so it's already out of sync. and lowercase the package.
package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ElbowIOInputsAutoLogged extends ElbowIO.ElbowIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ElbowRotateAngle", elbowRotateAngle);
    table.put("ElbowCurrent", elbowCurrent);
    table.put("ElbowVoltage", elbowVoltage);
    table.put("ElbowAngularV", angularVelocityRad);

    table.put("atMinAngle", atMinAngle);
    table.put("atMaxAngle", atMaxAngle);
  }

  @Override
  public void fromLog(LogTable table) {

    elbowVoltage = table.get("ElbowVoltage", elbowVoltage);
    elbowCurrent = table.get("ElbowCurrent", elbowCurrent);
    angularVelocityRad = table.get("ElbowAngularV", angularVelocityRad);
    elbowRotateAngle = table.get("ElbowRotateAngle", elbowRotateAngle);
    atMaxAngle = table.get("atMaxAngle", atMaxAngle);
    atMinAngle = table.get("atMinAngle", atMinAngle);
  }

  @Override
  public ElbowIOInputsAutoLogged clone() {
    ElbowIOInputsAutoLogged copy =
        new ElbowIOInputsAutoLogged();

    copy.elbowVoltage = elbowVoltage;
    copy.elbowCurrent = elbowCurrent;
    copy.angularVelocityRad = angularVelocityRad;

    copy.elbowRotateAngle = elbowRotateAngle;

    copy.atMaxAngle = atMaxAngle;
    copy.atMinAngle = atMinAngle;

    return copy;
  }
}