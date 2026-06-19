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