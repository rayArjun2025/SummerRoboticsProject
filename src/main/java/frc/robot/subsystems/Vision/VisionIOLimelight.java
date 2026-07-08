// Raymond: lowercase package - frc.robot.subsystems.vision.
package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs.PoseObservation;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs.PoseObservationType;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

public class VisionIOLimelight implements VisionIO {

  private final String leftCamera;
  private final String rightCamera;

  // Raymond: these camera names have to match what's actually configured on the limelights or you
  // get nothing back. pull them into VisionConstants instead of hardcoding strings here. also note
  // nothing builds this class yet (see the REAL case in Vision.java) so it's untested.
  public VisionIOLimelight() {
    leftCamera = "left-limelight";
    rightCamera = "right-limelight";
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {

    var leftResults = LimelightHelpers.getLatestResults(leftCamera);
    var rightResults = LimelightHelpers.getLatestResults(rightCamera);

    inputs.connected = leftResults != null || rightResults != null;

    if (!inputs.connected) {
      inputs.hasTarget = false;
      inputs.visibleTagIds = new int[0];
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    boolean leftHasTargets = LimelightHelpers.getTV(leftCamera);
    boolean rightHasTargets = LimelightHelpers.getTV(rightCamera);

    if (!leftHasTargets && !rightHasTargets) {
      inputs.hasTarget = false;
      inputs.visibleTagIds = new int[0];
      inputs.poseObservations = new PoseObservation[0];
      return;
    }

    inputs.hasTarget = true;

    String bestCamera = null;

    if (leftHasTargets) {
      bestCamera = leftCamera;
    }

    if (rightHasTargets) {

      if (bestCamera == null) {
        bestCamera = rightCamera;
      } else {

        // Raymond: you call it leftArea but you're reading getTA(bestCamera). it happens to be the
        // left one here so it works, but name it bestArea so it doesn't trip up the next reader.
        double leftArea = LimelightHelpers.getTA(bestCamera);
        double rightArea = LimelightHelpers.getTA(rightCamera);

        if (rightArea > leftArea) {
          bestCamera = rightCamera;
        }
      }
    }

    if (bestCamera != null) {
      inputs.targetXDegrees = LimelightHelpers.getTX(bestCamera);

      inputs.targetYDegrees = LimelightHelpers.getTY(bestCamera);

      inputs.targetAreaPercent = LimelightHelpers.getTA(bestCamera);
    }

    Set<Integer> tagIds = new HashSet<>();

    if (leftResults != null) {
      for (var target : leftResults.targets_Fiducials) {
        tagIds.add((int) target.fiducialID);
      }
    }

    if (rightResults != null) {
      for (var target : rightResults.targets_Fiducials) {
        tagIds.add((int) target.fiducialID);
      }
    }

    int[] ids = new int[tagIds.size()];
    int index = 0;

    for (int id : tagIds) {
      ids[index] = id;
      index++;
    }

    inputs.visibleTagIds = ids;

    List<PoseObservation> observations = new ArrayList<>();

    addObservation(observations, LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftCamera));

    addObservation(observations, LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightCamera));

    inputs.poseObservations = observations.toArray(new PoseObservation[0]);
  }

  private void addObservation(
      List<PoseObservation> observations, LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null) {
      return;
    }

    if (estimate.tagCount <= 0) {
      return;
    }

    observations.add(
        new PoseObservation(
            estimate.timestampSeconds,
            new Pose3d(estimate.pose),
            0.0,
            estimate.tagCount,
            estimate.avgTagDist,
            PoseObservationType.MEGATAG_2));
  }

  @Override
  public void setPipeline(int index) {
    LimelightHelpers.setPipelineIndex(leftCamera, index);
    LimelightHelpers.setPipelineIndex(rightCamera, index);
  }

  @Override
  public void setFiducialFilters(int[] ids) {
    LimelightHelpers.SetFiducialIDFiltersOverride(leftCamera, ids);
    LimelightHelpers.SetFiducialIDFiltersOverride(rightCamera, ids);
  }

  @Override
  public void setRobotOrientation(Rotation2d heading) {

    LimelightHelpers.SetRobotOrientation(leftCamera, heading.getDegrees(), 0, 0, 0, 0, 0);

    LimelightHelpers.SetRobotOrientation(rightCamera, heading.getDegrees(), 0, 0, 0, 0, 0);
  }

  @Override
  public void updatePose(edu.wpi.first.math.geometry.Pose2d robotPose) {}
}
