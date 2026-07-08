package frc.robot.subsystems.arm;

public class ArmConstants {
    public static final double SHOULDER_MAX_ANGLE = Math.toRadians(180);
    public static final double SHOULDER_MIN_ANGLE = -Math.toRadians(90);
    public static final double SHOULDER_ZERO_REF = Math.toRadians(90);
    public static final double SHOULDER_GEAR_RATIO = 150;
    public static final double SHOULDER_ARM_LENGTH = 0.7366;
    public static final double SHOULDER_ARM_MASS = 5.669;
    public static final int SHOULDER_MOTOR_ID = 1;

    public static final double SHOULDER_KP = 40.0;
    public static final double SHOULDER_KI = 0.0;
    public static final double SHOULDER_KD = 1.0;
    public static final double SHOULDER_KS = 0.2;
    public static final double SHOULDER_KV = 0.12;
    public static final double SHOULDER_KA = 0.01;
    public static final double SHOULDER_MOTION_MAGIC_JERK = 20;
    public static final double SHOULDER_MOTION_MAGIC_ACCELERATION = 2;
    public static final double SHOULDER_MOTION_MAGIC_CRUISE_VELOCITY = 1;
    

    public static final double ELBOW_MAX_ANGLE = Math.toRadians(90);
    public static final double ELBOW_MIN_ANGLE = -Math.toRadians(90);
    public static final double ELBOW_ZERO_REF = Math.toRadians(0);
    public static final double ELBOW_GEAR_RATIO = 105;
    public static final double ELBOW_ARM_LENGTH = 0.8128;
    public static final double ELBOW_ARM_MASS = 3.2;
    public static final double ELBOW_KP = 45.0;
    public static final double ELBOW_KI = 0.0;
    public static final double ELBOW_KD = 1.2;
    public static final double ELBOW_KS = 0.20;
    public static final double ELBOW_KV = 0.12;
    public static final double ELBOW_KA = 0.01;
    public static final int ELBOW_MOTOR_ID = 2;
    public static final double ELBOW_MOTION_MAGIC_CRUISE_VELOCITY = 0;
    public static final double ELBOW_MOTION_MAGIC_ACCELERATION = 0;
    public static final double ELBOW_MOTION_MAGIC_JERK = 0;

    public static final double UPDATE_RATE = 50.0;
    public static final double ARM_KG = 0.35;
    public static final double LOW_CLAMP = -12;
    public static final double HIGH_CLAMP = 12;
    public static final double GRAVITY_FF = 2.5;
    public static final double TOLERANCE_RAD = 0.01;
    
}
