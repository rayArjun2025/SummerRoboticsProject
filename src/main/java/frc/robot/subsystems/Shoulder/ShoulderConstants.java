// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
package frc.robot.subsystems.Shoulder;

// Raymond: make this final, it's a constants holder - nobody should extend or instantiate it.
public class ShoulderConstants {
  public static final double MAX_ANGLE = Math.toRadians(180);
  public static final double MIN_ANGLE = -Math.toRadians(90);
  public static final double ZERO_REF = Math.toRadians(90);
  // Raymond: 2.5 FF can live here, but the stuff still hardcoded in the other files belongs here
  // too - kP/kI/kD, the 1.0 tolerance, the 50hz update rate, and the motor current limits (we run
  // 60 supply / 82 stator).
  public static final double GRAVITY_FF = 2.5;
  public static final double GEAR_RATIO = 150;
  public static final double ARM_LENGTH = 0.7366;
  public static final double ARM_MASS = 5.669;
  public static final double CHANGE_IN_TIME = 0.02;
  // Raymond: CAN id 1 is a placeholder and it'll collide with other subsystems' motors on the bus.
  // set the real id.
  public static final int MOTOR_ID = 1;
}
