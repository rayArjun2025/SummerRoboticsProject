// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
        public double hookOutputCurrent = 0.0;
        public double hookOutputVoltage = 0.0;
        public double hookPosition = 0.0;
        public double hookVelocity = 0.0;

        public double wheelOutputCurrent = 0.0;
        public double wheelOutputVoltage = 0.0;
        public double wheelPosition = 0.0;
        public double wheelVelocity = 0.0;
    }

  public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setHookVoltage(double volts_V, double ff_V) {}

    public default void setHookVelocity(double velocity_rps) {}

    public default void setWheelVoltage(double volts_V, double ff_V) {}

    public default void setWheelVelocity(double velocity_rps) {}

    public default void stopClimb() {}

    public default void climbTo(double position) {}

    public default void putUpTheWheels(double position) {}

    public default void toggleBrake() {}

}
