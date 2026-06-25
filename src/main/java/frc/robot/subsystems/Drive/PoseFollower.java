// Raymond: lowercase package - frc.robot.subsystems.drive, rename the folder. this file diverges a lot from the reference follower (no accel limiting / ramping / hang mode) - that's fine if this robot doesn't need it, but a couple real problems below.
package frc.robot.subsystems.Drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Util;

public class PoseFollower {

    private double translate_kP = 3.0;
    private double rotate_kP = 2.0;

    // Raymond: reference seeds this from Drive.MAX_LINEAR_VEL_mps so the cap matches what the drivetrain actually uses. you reach into TunerConstants instead - and Drive.MAX_LINEAR_VEL_mps is hardcoded to 4.8, not kSpeedAt12Volts, so these two now disagree. use the Drive constant.
    private double maxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private Pose2d targetPose = new Pose2d();

    public PoseFollower(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP) {
        setParams(targetPose, maxSpeed, translate_kp, rotate_kP);
    }

    public PoseFollower(Pose2d targetPose, double maxSpeed) {
        this(targetPose, maxSpeed, 3.0, 2.0);
    }

    public void setParams(Pose2d targetPose, double maxSpeed){
        setParams(targetPose, maxSpeed, translate_kP, rotate_kP);
    }

    public void setParams(Pose2d targetPose, double maxSpeed, double translate_kp, double rotate_kP){
        this.targetPose = targetPose;
        this.maxSpeed = maxSpeed;
        this.translate_kP = translate_kp;
        this.rotate_kP = rotate_kP;
    }

    // Raymond: reference is process(Pose2d currentPose) - the caller hands in the pose. yours takes no arg and grabs Drive.getInstance().getPose() itself. that singleton reach-in makes this impossible to unit test and couples the follower to Drive. take the pose as a param.
    public ChassisSpeeds process() {
        Transform2d poseDiff = targetPose.minus(Drive.getInstance().getPose()); // Calculate the difference in position.

        double magnitude = Math.hypot(
                poseDiff.getX(),
                poseDiff.getY()); // Calculate the magnitude (distance to the target).

        Transform2d direction; // This will hold the normalized and limited vector.

        Rotation2d rotationDiff = poseDiff.getRotation();
        double radDiff = rotationDiff.getRadians();
        double angularVelocity = Util.limit(rotate_kP * radDiff, -Drive.getInstance().MAX_ANGULAR_VEL_radps,
                Drive.getInstance().MAX_ANGULAR_VEL_radps);

        // Raymond: this line is a bug. you just limited angularVelocity to MAX_ANGULAR_VEL_radps (rad/s), then multiply by radiansToDegrees(1) (~57.3), blowing it ~57x past the limit and out of units. ChassisSpeeds.omega wants rad/s - delete this line. reference has nothing like it.
        angularVelocity *= Units.radiansToDegrees(1);

        double output = Util.limit(translate_kP * magnitude, maxSpeed);
        if(magnitude < 0.05){
            output = 0;
        }

        direction = poseDiff.times(output/magnitude);

        // Create the ChassisSpeeds with the limited linear velocities.
        ChassisSpeeds speeds = new ChassisSpeeds(
                direction.getX(), // Linear velocity in the x-direction (vx)
                direction.getY(), // Linear velocity in the y-direction (vy)
                angularVelocity // No angular velocity (omega)
        );

        return speeds;
    }

}