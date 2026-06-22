package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs{
        public boolean connected = false;
        public boolean hasTarget = false;

        public double targetXDegrees = 0.0;
        public double targetYDegrees = 0.0;
        public double targetAreaPercent = 0.0;

        public int[] visibleTagIds = new int[0];
        public int tagCount = 0;

        public Pose2d estimatedPose = new Pose2d();
        public double timestampSeconds = 0.0;

      
        public double avgTagDistance = 0.0;

         public PoseObservation[] poseObservations = new PoseObservation[0];

        public static record PoseObservation(double timestamp, Pose3d pose, double ambiguity,
            int tagCount, double avgTagDistance, PoseObservationType type) {
        }

        public enum PoseObservationType {
            MEGATAG_1, MEGATAG_2, PHOTONVISION
        }
    }

    
    public default void updateInputs(VisionIOInputs inputs) {}
    public default void setPipeline(int mode) {}
    public default void updatePose(Pose2d robotPose) {}
    public default void setRobotOrientation(Rotation2d rotation) {}
    
}
