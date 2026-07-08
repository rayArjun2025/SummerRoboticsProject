// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
package frc.robot.subsystems.Shoulder;

import org.littletonrobotics.junction.AutoLog;

public interface ShoulderIO {

  @AutoLog
  public static class ShoulderIOInputs {
    // Raymond: put unit suffixes on these like angularVelocityRad already has -
    // shoulderSwivelAngle_rad, shoulderCurrent_amps, shoulderVoltage_volts. half of them carry
    // units and half don't.
    public double shoulderSwivelAngle = 0;
    public double angularVelocityRad = 0;
    public double shoulderCurrent = 0;
    public double shoulderVoltage = 0;
    public boolean atMaxAngle = false;
    public boolean atMinAngle = false;
    public boolean connected = false;
  }

  public default void updateInputs(ShoulderIOInputs inputs) {}

  public default void setShoulderVoltage(double volts) {}

  public default void stopMotor() {}
}
