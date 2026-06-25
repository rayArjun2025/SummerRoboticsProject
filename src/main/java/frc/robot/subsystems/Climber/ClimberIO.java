// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: lowercase package - frc.robot.subsystems.climber.
package frc.robot.subsystems.Climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  // Raymond: indentation is all over the place in this file - 2 spaces here, 4 spaces below, 8 on the fields. run the formatter.
  @AutoLog
  public static class ClimberIOInputs {
        public double hookOutputCurrent = 0.0;
        public double hookOutputVoltage = 0.0;
        public double hookPositionDeg = 0.0;
        // Raymond: every other field has a unit but velocity doesn't. it's degrees/sec from the IO so call it hookVelocity_dps. same for wheelVelocity.
        public double hookVelocity = 0.0;

        public double wheelOutputCurrent = 0.0;
        public double wheelOutputVoltage = 0.0;
        public double wheelPositionDeg = 0.0;
        public double wheelVelocity = 0.0;
    }

  public default void updateInputs(ClimberIOInputs inputs) {}

    public default void setHookVoltage(double volts_V, double ff_V) {}

    public default void setHookVelocity(double velocity_rps) {}

    public default void setWheelVoltage(double volts_V, double ff_V) {}

    public default void setWheelVelocity(double velocity_rps) {}

    public default void stopClimb() {}

    // Raymond: param names use snake_case here but camelCase in the Real impl (hook_position_deg). pick camelCase, that's what we use in java. and add the _deg unit.
    public default void climbTo(double hook_position, double wheel_position) {}

    // Raymond: putUpTheWheels is never called anywhere and no IO implements it. dead method, delete it.
    public default void putUpTheWheels(double position) {}

    // Raymond: toggleBrake is declared but no IO implements it, so it does nothing. either implement setBrake(boolean) in Real or remove it. a no-op toggle is worse than nothing.
    public default void toggleBrake() {}

}
