// Raymond: lowercase package - frc.robot.subsystems.vision.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean connected = false;
    public boolean hasTarget = false;

    public double targetXDegrees = 0.0;
    public double targetYDegrees = 0.0;
    public double targetAreaPercent = 0.0;

    public int[] visibleTagIds = new int[0];
    public int tagCount = 0;

    // Raymond: you added estimatedPose/timestampSeconds/avgTagDistance/tagCount on top of the
    // reference inputs, but nothing fills them in and they're not in VisionIOInputsAutoLogged's
    // toLog/fromLog. so they never log and never replay - dead fields. either wire them up
    // everywhere or drop them. the data you actually use already lives in poseObservations.
    public Pose2d estimatedPose = new Pose2d();
    public double timestampSeconds = 0.0;

    public double avgTagDistance = 0.0;

    public PoseObservation[] poseObservations = new PoseObservation[0];

    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double avgTagDistance,
        PoseObservationType type) {}

    public enum PoseObservationType {
      MEGATAG_1,
      MEGATAG_2,
      PHOTONVISION
    }
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public default void setPipeline(int mode) {}

  public default void updatePose(Pose2d robotPose) {}

  public default void setRobotOrientation(Rotation2d rotation) {}

  public default void setFiducialFilters(int[] ids) {}
}
