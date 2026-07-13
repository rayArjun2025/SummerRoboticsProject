package frc.robot.subsystems.arm;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class ArmIOInputsAutoLogged extends ArmIO.ArmIOInputs
    implements LoggableInputs, Cloneable {

  @Override
  public void toLog(LogTable table) {
    table.put("ShoulderSwivelAngle_rad", shoulderSwivelAngle_rad);
    table.put("ElbowSwivelAngle_rad", elbowSwivelAngle_rad);

    table.put("ElbowAV_rad", elbowAV_rad);
    table.put("ShoulderAV_rad", shoulderAV_rad);

    table.put("ShoulderCurrent_amps", shoulderCurrent_amps);
    table.put("ElbowCurrent_amps", elbowCurrent_amps);

    table.put("ShoulderVoltage_volts", shoulderVoltage_volts);
    table.put("ElbowVoltage_volts", elbowVoltage_volts);

    table.put("Connected", connected);
  }

  @Override
  public void fromLog(LogTable table) {
    shoulderSwivelAngle_rad = table.get("ShoulderSwivelAngle_rad", shoulderSwivelAngle_rad);
    elbowSwivelAngle_rad = table.get("ElbowSwivelAngle_rad", elbowSwivelAngle_rad);

    elbowAV_rad = table.get("ElbowAV_rad", elbowAV_rad);
    shoulderAV_rad = table.get("ShoulderAV_rad", shoulderAV_rad);

    shoulderCurrent_amps = table.get("ShoulderCurrent_amps", shoulderCurrent_amps);
    elbowCurrent_amps = table.get("ElbowCurrent_amps", elbowCurrent_amps);

    shoulderVoltage_volts = table.get("ShoulderVoltage_volts", shoulderVoltage_volts);
    elbowVoltage_volts = table.get("ElbowVoltage_volts", elbowVoltage_volts);

    connected = table.get("Connected", connected);
  }

  @Override
  public ArmIOInputsAutoLogged clone() {
    ArmIOInputsAutoLogged copy = new ArmIOInputsAutoLogged();

    copy.shoulderSwivelAngle_rad = this.shoulderSwivelAngle_rad;
    copy.elbowSwivelAngle_rad = this.elbowSwivelAngle_rad;

    copy.elbowAV_rad = this.elbowAV_rad;
    copy.shoulderAV_rad = this.shoulderAV_rad;

    copy.shoulderCurrent_amps = this.shoulderCurrent_amps;
    copy.elbowCurrent_amps = this.elbowCurrent_amps;

    copy.shoulderVoltage_volts = this.shoulderVoltage_volts;
    copy.elbowVoltage_volts = this.elbowVoltage_volts;

    copy.connected = this.connected;

    return copy;
  }
}