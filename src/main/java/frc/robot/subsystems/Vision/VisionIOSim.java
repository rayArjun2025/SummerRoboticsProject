package frc.robot.subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

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

        lPoseEstimator = new PhotonPoseEstimator(reefField, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, leftTransform3d);
        rPoseEstimator = new PhotonPoseEstimator(reefField, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, rightTransform3d);

        robotPose2d = new Pose2d();
    }
    



}
