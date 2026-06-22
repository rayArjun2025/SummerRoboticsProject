package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs.PoseObservation;
import frc.robot.subsystems.Vision.VisionIO.VisionIOInputs.PoseObservationType;

public class VisionIOSim implements VisionIO{

    private final AprilTagFieldLayout reefField;
    private final VisionSystemSim visionSys;
    private final PhotonCameraSim leftCameraSim;
    private final PhotonCameraSim rightCameraSim;

    private final PhotonCamera leftCamera;
    private final PhotonCamera rightCamera;

    private final Transform3d leftTransform3d;
    private final Transform3d rightTransform3d;

    private final PhotonPoseEstimator lPoseEstimator;
    private final PhotonPoseEstimator rPoseEstimator;

    private Pose2d robotPose2d;
    

    public VisionIOSim(Transform3d leftTransform3d, Transform3d rightTransform3d){

        this.leftTransform3d = leftTransform3d;
        this.rightTransform3d = rightTransform3d;

        visionSys = new VisionSystemSim("mainVis");
        reefField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        visionSys.addAprilTags(reefField);

        leftCamera = new PhotonCamera("left-Camera");
        rightCamera = new PhotonCamera("right-Camera");

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 880, Rotation2d.fromDegrees(70));
        props.setFPS(90);
        props.setAvgLatencyMs(11);
        props.setLatencyStdDevMs(2);

        leftCameraSim = new PhotonCameraSim(leftCamera, props);
        rightCameraSim = new PhotonCameraSim(rightCamera, props);

        visionSys.addCamera(leftCameraSim, leftTransform3d);
        visionSys.addCamera(rightCameraSim, rightTransform3d);

        lPoseEstimator = new PhotonPoseEstimator(reefField, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.leftTransform3d);
        rPoseEstimator = new PhotonPoseEstimator(reefField, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, this.rightTransform3d);

        robotPose2d = new Pose2d();
    }

    @Override
    public void updatePose(Pose2d robotPose) {  
        robotPose2d = robotPose;
    }
    
    @Override
    public void updateInputs(VisionIOInputs inputs) {

        visionSys.update(robotPose2d);

        PhotonPipelineResult leftResult = leftCamera.getLatestResult();
        PhotonPipelineResult rightResult = rightCamera.getLatestResult();

        
        inputs.connected = (leftResult != null || rightResult != null);;


        boolean leftHasTargets = leftResult != null && leftResult.hasTargets();
        boolean rightHasTargets = rightResult != null && rightResult.hasTargets();

       
        if (!leftHasTargets && !rightHasTargets) {
            inputs.hasTarget = false;
            inputs.visibleTagIds = new int[0];
            inputs.poseObservations = new PoseObservation[0];
            return;
        }

        inputs.hasTarget = true;

        PhotonTrackedTarget bestTarget = null;

        if (leftHasTargets) {
            bestTarget = leftResult.getBestTarget();
        }

        if (rightHasTargets) {
            PhotonTrackedTarget rightBest = rightResult.getBestTarget();
            if (bestTarget != null && rightBest != null) {

                double leftDistance =
                    bestTarget.getBestCameraToTarget().getTranslation().getNorm();

                double rightDistance =
                    rightBest.getBestCameraToTarget().getTranslation().getNorm();

                if (rightDistance < leftDistance) {
                    bestTarget = rightBest;
                }

            } else if (bestTarget == null) {
                bestTarget = rightBest;
            }
        }

        if (bestTarget != null) {
            inputs.targetXDegrees = bestTarget.getYaw();
            inputs.targetYDegrees = bestTarget.getPitch();
            inputs.targetAreaPercent = bestTarget.getArea();
        }

        Set<Integer> tagIds = new HashSet<>();

        if (leftHasTargets) {
            for (PhotonTrackedTarget t : leftResult.getTargets()) {
                tagIds.add(t.getFiducialId());
            }
        }

        if (rightHasTargets) {
            for (PhotonTrackedTarget t : rightResult.getTargets()) {
                tagIds.add(t.getFiducialId());
            }
        }

        int[] ids = new int[tagIds.size()];
        int i = 0;
        for (int id : tagIds) ids[i++] = id;
        inputs.visibleTagIds = ids;


        List<PoseObservation> observations = new ArrayList<>();

        if (leftHasTargets) {
            var leftPose = lPoseEstimator.update(leftResult);
            if (leftPose.isPresent()) {
                addObservation(observations, leftPose.get());
            }
        }

        if (rightHasTargets) {
            var rightPose = rPoseEstimator.update(rightResult);
            if (rightPose.isPresent()) {
                addObservation(observations, rightPose.get());
            }
        }

        inputs.poseObservations =
            observations.toArray(PoseObservation[]::new);
    }

    public void addObservation(List<PoseObservation> observations, EstimatedRobotPose est) {

        if (est == null || est.targetsUsed.isEmpty()) {
            return;
        }

        double avgDist =
            est.targetsUsed.stream()
                .mapToDouble(t ->
                    t.getBestCameraToTarget()
                    .getTranslation()
                    .getNorm())
                .average()
                .orElse(Double.POSITIVE_INFINITY);

        boolean megaTag1 = avgDist < 3.6;

        observations.add(
            new PoseObservation(
                est.timestampSeconds,
                est.estimatedPose,
                0.0,
                est.targetsUsed.size(),
                avgDist,
                megaTag1
                    ? PoseObservationType.MEGATAG_1
                    : PoseObservationType.MEGATAG_2));
    }

    @Override
    public void setPipeline(int mode) {}

    @Override
    public void setRobotOrientation(Rotation2d rotation) {}

}
