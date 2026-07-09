// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

public final class ClimberConstants {
  // Raymond: CAN IDs 0 and 1 are almost certainly going to collide with other devices - 0 is a
  // really common default. set the real IDs and leave a // placeholder comment like reference does
  // until you know them.
  public static final int hookMotorID = 0;
  public static final int wheelMotorID = 1;

  public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
  public static final double STATOR_CURRENT_LIMIT_A = 82.0;

  public static final double tolerance_deg = 1.0;
  public static final double maxVoltage = 12.0;

  
  public static final double homingDegrees_deg = 0.0;
  public static final double MIN_DEG = 0;
  public static final double MAX_DEG = 0;

  public static final double GEAR_RATIO = 50.0;
  public static final double ARM_LENGTH = 0.22;
  public static final double ARM_MASS = 1.8;
  public static final double LOW_CLAMP = -12;
  public static final double HIGH_CLAMP = 12;
  
  public static final double climberKP = 4.0;
  public static final double climberKI = 0;
  public static final double climberKD = 0.4;
  public static final double climberKV = 1;
  public static final double climberKS = 0.18;
  public static final double climberKA = 0.08;
  public static final double climberKG = 0.35;

  public static final double MAX_VELOCITY = Math.toRadians(180);   // rad/s
  public static final double MAX_ACCELERATION = Math.toRadians(360); // rad/s^2



  

  // Raymond: this is where the tolerance and the target/home angles belong (TOLERANCE_DEG, etc),
  // not hardcoded as 1.0 / 90.0 inside Climber.java. also missing gear ratio and current limits -
  // right now the limits are magic numbers buried in ClimberIOReal. constants file is the one place
  // all the tuning lives.
}