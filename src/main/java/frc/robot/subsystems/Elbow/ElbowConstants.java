package frc.robot.subsystems.Elbow;

// Raymond: lowercase package - frc.robot.subsystems.elbow.
// Raymond: make this class final, all our constants classes are (look at ClimberConstants in the real repo). also this is where the missing constants belong: PID gains (kP/kI/kD), the tolerance, the voltage clamp, and the current limits for the real motor.
public class ElbowConstants {
    public static final double MAX_ANGLE = Math.toRadians(90);
    public static final double MIN_ANGLE = -Math.toRadians(90);
    public static final double ZERO_REF = Math.toRadians(0);
    public static final double GRAVITY_FF = 2.5;
    public static final double GEAR_RATIO = 105;
    public static final double ARM_LENGTH = 0.8128;
    public static final double ARM_MASS = 3.2;
    public static final double CHANGE_IN_TIME = 0.02;
    // Raymond: these are placeholder CAN IDs and they're BOTH 1 - that'll collide on the bus the second you also give something else id 1, and a motor and cancoder can't share an id anyway. set the real ids.
    public static final int MOTOR_ID = 1;
    // Raymond: also nothing actually uses CANCODER_ID - ElbowReal never makes a CANCoder. either wire up the cancoder or drop this and the offset.
    public static final int CANCODER_ID = 1;
    public static final double CANCODER_OFFSET = 0.0;
}
