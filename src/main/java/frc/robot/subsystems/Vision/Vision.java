// Raymond: capitalized package again - all our packages are lowercase. should be
// frc.robot.subsystems.vision, and rename the Vision/ folder to vision/ to match the reference.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Vision {
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
  private static Vision instance;

  // Raymond: these bare numbers mean nothing to the next person. label them - x/y/z meters and the
  // mount angles - or better, pull the camera offsets into VisionConstants. the reference keeps the
  // offset in one named transform for exactly this reason.
  public static final Transform3d LEFT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(0.0889, 0.2794, 0.4445),
          new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(90)));

  public static final Transform3d RIGHT_ROBOT_TO_CAMERA =
      new Transform3d(
          new Translation3d(0.0889, -0.2794, 0.4445),
          new Rotation3d(0.0, Units.degreesToRadians(20), Units.degreesToRadians(-90)));

  public Vision(VisionIO io) {
    this.io = io;
  }

  public static Vision getInstance() {
    if (instance == null) {
      switch (Constants.currentMode) {
          // Raymond: this is the big one. REAL is running VisionIOSim - so on the actual robot
          // vision
          // runs in simulation and never reads the real limelights. REAL has to be new
          // VisionIOLimelight(), like the reference. right now your whole VisionIOLimelight class
          // is
          // dead code because nothing ever builds it.
        case REAL:
          instance = new Vision(new VisionIOSim(LEFT_ROBOT_TO_CAMERA, RIGHT_ROBOT_TO_CAMERA));
          break;
        case SIM:
          instance = new Vision(new VisionIOSim(LEFT_ROBOT_TO_CAMERA, RIGHT_ROBOT_TO_CAMERA));
          break;
        case REPLAY:
          break;

        default:
          break;
      }
    }
    return instance;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);
  }

  // Raymond: inputs.estimatedPose is never set by either IO (limelight or sim), so this always
  // hands back a blank Pose2d. either populate it in updateInputs or use poseObservations[0].pose()
  // like the reference does - right now anything calling this gets a lie.
  public Pose2d getEstimatedPose() {
    return inputs.estimatedPose;
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public double getTargetX() {
    return inputs.targetXDegrees;
  }

  public double getTargetY() {
    return inputs.targetYDegrees;
  }

  public double getTargetArea() {
    return inputs.targetAreaPercent;
  }

  public void setRobotOrientation(Rotation2d rotation) {
    io.setRobotOrientation(rotation);
  }
}
