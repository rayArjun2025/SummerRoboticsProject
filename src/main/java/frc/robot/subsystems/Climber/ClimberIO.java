// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberIOInputs {
        public double motor1OutputCurrent = 0.0;
        public double motor1OutputVoltage = 0.0;
        public double motor1Position = 0.0;
        public double motor1Velocity = 0.0;

        public double motor2OutputCurrent = 0.0;
        public double motor2OutputVoltage = 0.0;
        public double motor2Position = 0.0;
        public double motor2Velocity = 0.0;
    }

  public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setMotor1Voltage(double volts_V, double ff_V) {}

    public default void setMotor1Velocity(double velocity_rps) {}

    public default void setMotor2Voltage(double volts_V, double ff_V) {}

    public default void setMotor2Velocity(double velocity_rps) {}

    public default void stopClimb() {}

    public default void climbTo(double position) {}

    public default void putUpTheWheels(double position) {}

    public default void toggleBrake() {}

}
