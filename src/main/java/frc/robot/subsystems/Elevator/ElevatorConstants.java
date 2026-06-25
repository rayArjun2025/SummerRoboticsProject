package frc.robot.subsystems.Elevator;

// Raymond: make this final - it's a constants holder, nobody should extend it. reference ones are all `public final class`.
public class ElevatorConstants {
    // Raymond: this is a good start, but the magic numbers I flagged in Elevator.java and the IO files (PID gains, voltage clamp, position tolerance, current limit, update freq) all belong in here too.
    public static final double ELEVATOR_MAX_HEIGHT = 2.0;
    public static final double ELEVATOR_MIN_HEIGHT = 0.0;
    public static final double CHANGE_IN_TIME = 0.02;
    public static final double DRUM_RADIUS = 0.01905; // 0.75 in
    public static final double MAX_RPS = 100;
    public static final double GRAVITY_FF = 1.5;
    public static final double CARRIAGE_MASS = 7.5;
    public static final double GEAR_RATIO = 5.5;
    // Raymond: MOTOR_ID = 1 is a placeholder and it'll collide - everyone's leaving CAN IDs at 0/1. set the real ID and add a // placeholder note like the reference does until you know it.
    public static final int MOTOR_ID = 1;

}