// Raymond: lowercase package.
package frc.robot.subsystems.Hand;

// Raymond: make it `public final class` like ClimberConstants - nobody should extend a constants holder. also the grip target angle, home angle and tolerance belong in here, not as magic numbers in Hand.java. and trim the pile of blank lines at the bottom.
public class HandConstants {
    
    public static final int motorID = 0; // Raymond: CAN ID 0 collides - Climber also uses 0. two devices can't share an ID. set real IDs.
    
    public static final double kP = 1.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;

    




}