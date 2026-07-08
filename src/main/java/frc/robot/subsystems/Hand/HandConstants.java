package frc.robot.subsystems.hand;

public final class HandConstants {

  public static final int motorID = 0;
  // Raymond: CAN ID 0 collides - Climber also uses 0. two devices can't share an ID. set
  // real IDs.

  public static final double coralTarget_deg = 90.0;
  public static final double algaeTarget_deg = 80.0;
  public static final double home_deg = 0.0;
  public static final double tolerance_deg = 1.0;
  public static final double maxVoltage = 12.0;

  public static final double SUPPLY_CURRENT_LIMIT_A = 60.0;
  public static final double STATOR_CURRENT_LIMIT_A = 82.0;

  public static final double kP = 3.0;
  public static final double kI = 0.0;
  public static final double kD = 0.1;
  public static final double kS = 0.08;
  public static final double kA = 0.02;
  public static final double kV = 0.25;

  public static final double LOW_CLAMP = -12.0;
  public static final double HIGH_CLAMP = 12.0;
}
