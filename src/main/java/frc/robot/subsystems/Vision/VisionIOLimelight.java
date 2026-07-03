package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs.PoseObservationType;

public class VisionIOLimelight implements VisionIO {

    private final String leftCamera;
    private final String rightCamera;

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

                double leftArea = LimelightHelpers.getTA(leftCamera);
                double rightArea = LimelightHelpers.getTA(rightCamera);

                if (rightArea > leftArea) {
                    bestCamera = rightCamera;
                }
            }
        }

        if (bestCamera != null) {
            inputs.targetXDegrees =
                LimelightHelpers.getTX(bestCamera);

            inputs.targetYDegrees =
                LimelightHelpers.getTY(bestCamera);

            inputs.targetAreaPercent =
                LimelightHelpers.getTA(bestCamera);
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

        addObservation(
            observations,
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(leftCamera));

        addObservation(
            observations,
            LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(rightCamera));

        inputs.poseObservations =
            observations.toArray(new PoseObservation[0]);
    }

    private void addObservation(List<PoseObservation> observations, LimelightHelpers.PoseEstimate estimate) {
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
                PoseObservationType.MEGATAG_2
            )
        );
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

        LimelightHelpers.SetRobotOrientation(
            leftCamera,
            heading.getDegrees(),
            0, 0, 0, 0, 0);

        LimelightHelpers.SetRobotOrientation(
            rightCamera,
            heading.getDegrees(),
            0, 0, 0, 0, 0);
    }

    @Override
    public void updatePose(edu.wpi.first.math.geometry.Pose2d robotPose) {}
}