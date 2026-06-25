// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: lowercase package.
package frc.robot.subsystems.Climber;

public final class ClimberConstants {
    // Raymond: CAN IDs 0 and 1 are almost certainly going to collide with other devices - 0 is a really common default. set the real IDs and leave a // placeholder comment like reference does until you know them.
    public static final int hookMotorID = 0;
    public static final int wheelMotorID = 1;

    // Raymond: don't declare two constants on one line. one per line so they diff cleanly and you can comment each.
    public static final double hookKP = 1.0, wheelKP = 1.0;
    public static final double hookKI = 0.0, wheelKI = 0.0;
    public static final double hookKD = 0.0, wheelKD = 0.0;
    public static final double hookKS = 0.0, wheelKS = 0.0;
    public static final double hookKV = 0.0, wheelKV = 0.0;
    public static final double hookKA = 0.0, wheelKA = 0.0;
    // Raymond: this is where the tolerance and the target/home angles belong (TOLERANCE_DEG, etc), not hardcoded as 1.0 / 90.0 inside Climber.java. also missing gear ratio and current limits - right now the limits are magic numbers buried in ClimberIOReal. constants file is the one place all the tuning lives.
}