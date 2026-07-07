// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

public final class ClimberConstants {
    // Raymond: CAN IDs 0 and 1 are almost certainly going to collide with other devices - 0 is a really common default. set the real IDs and leave a // placeholder comment like reference does until you know them.
    public static final int hookMotorID = 0;
    public static final int wheelMotorID = 1;

    public static final double loopPeriodSecs = 0.02;
    public static final double tolerance_deg = 1.0;
    public static final double maxVoltage = 12.0;

    public static final double targetDegrees_deg = 90.0;
    public static final double homeDegrees_deg = 0.0;

    public static final double hookKP = 1.0;
    public static final double hookKI = 0.0;
    public static final double hookKD = 0.0;
    public static final double hookKS = 0.0;
    public static final double hookKV = 0.0;
    public static final double hookKA = 0.0;

    public static final double wheelKP = 1.0;
    public static final double wheelKI = 0.0;
    public static final double wheelKD = 0.0;
    public static final double wheelKS = 0.0;
    public static final double wheelKV = 0.0;
    public static final double wheelKA = 0.0;
    // Raymond: this is where the tolerance and the target/home angles belong (TOLERANCE_DEG, etc), not hardcoded as 1.0 / 90.0 inside Climber.java. also missing gear ratio and current limits - right now the limits are magic numbers buried in ClimberIOReal. constants file is the one place all the tuning lives.
}