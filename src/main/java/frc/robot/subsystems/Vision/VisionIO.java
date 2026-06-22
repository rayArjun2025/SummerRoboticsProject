package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;

public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs{
        public boolean hasTarget = false;
        public int targetID  = -1;
        public double tx = 0.0;
        public double ty = 0.0;
        public Pose2d robotPose = new Pose2d();
    }

    
    public default void updateInputs(VisionIOInputs inputs) {}
    public default void setPipeline(int mode) {}
    
}
