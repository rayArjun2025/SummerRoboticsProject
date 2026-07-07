package frc.robot.subsystems.elbow;

import edu.wpi.first.units.measure.Frequency;

public class ElbowConstants {
    public static final double MAX_ANGLE = Math.toRadians(90);
    public static final double MIN_ANGLE = -Math.toRadians(90);
    public static final double ZERO_REF = Math.toRadians(0);
    public static final double GRAVITY_FF = 2.5;
    public static final double GEAR_RATIO = 105;
    public static final double ARM_LENGTH = 0.8128;
    public static final double ARM_MASS = 3.2;
    public static final double KP = 45.0;
    public static final double KI = 0.0;
    public static final double KD = 1.2;
    public static final double KS = 0.20;
    public static final double KV = 0.12;
    public static final double KA = 0.01;
    public static final double MAX_VOLTAGE = 12.0;;
    public static final double MIN_VOLTAGE = -12.0;
    public static final int MOTOR_ID = 1;
    public static final double TOLERANCE = 1.0;
    public static final double MOTION_MAGIC_CRUISE_VELOCITY = 0;
    public static final double MOTION_MAGIC_ACCELERATION = 0;
    public static final double MOTION_MAGIC_JERK = 0;
    public static final double KG = 0.35;
    public static final double UPDATE_RATE = 50.0;
}
