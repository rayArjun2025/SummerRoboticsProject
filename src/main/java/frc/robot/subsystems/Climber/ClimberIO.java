// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
    public double climberCurrent = 0.0; // measured in amps (A)
    public double climberVoltage = 0.0; // measured in volts (v)
    public double climberPositionDeg = 0.0; // underscore before units
    public double climberVelocity_dps = 0.0;
    public boolean connected = false;
  }

  public default void updateInputs(ClimberIOInputs inputs) {}

  public default void setClimberVoltage(double volts_V) {}

  public default void setClimberVelocity(double velocity_rps) {}

  public default void stopClimb() {}

  public default void zeroPosition() {}

  public default void climbTo() {}
  
  public default void setTargetAngle(double targetAngle_RAD) {}
}
