package frc.robot.subsystems.elevator;

public class ElevatorConstants {
    public static final double ELEVATOR_MAX_HEIGHT = 2.0;
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double LevelOneTargetHeight_m = 0.46;
    public static final double LevelTwoTargetHeight_m = 0.81;
    public static final double LevelThreeTargetHeight_m = 1.21;
    public static final double LevelFourTargetHeight_m = 1.83;

    public static final double DRUM_RADIUS = 0.01905; // 0.75 in
    public static final double MAX_RPS = 100;
    public static final double CARRIAGE_MASS = 7.5;
    public static final double GEAR_RATIO = 5.5;

    public static final double KP = 18.0;
    public static final double KI = 0.0;
    public static final double KD = 0.4;

    public static final double KS = 0.15;
    public static final double KV = 0.10;
    public static final double KA = 0.01;
    public static final double KG = 0.35;

    public static final double LOW_CLAMP = -12;
    public static final double HIGH_CLAMP = 12;

    public static final double TOLERANCE_METERS = 0.001;
    public static final double UPDATE_RATE = 50;

    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 2.0;
    public static final double MOTION_MAGIC_ACCELERATION = 8.0;
    public static final double MOTION_MAGIC_JERK = 80.0;

    public static final int MOTOR_ID = 3;

    public static final double MAX_VELOCITY = 1.5;
    public static final double MAX_ACCELERATION = 3.0;

    public static final double CoralHeight_m = 0.45;
    public static final double AlgaeHeight_m = 0.85;
}