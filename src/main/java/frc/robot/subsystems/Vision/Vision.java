package frc.robot.subsystems.Vision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class Vision {
    private final VisionIO io;
    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();
    private static Vision instance;

    public static final Transform3d LEFT_ROBOT_TO_CAMERA =
    new Transform3d(
        new Translation3d(
            0.0889,  
            0.2794,  
            0.4445   
        ),
        new Rotation3d(
            0.0,
            Units.degreesToRadians(20), 
            Units.degreesToRadians(90)   
        )
    );

    public static final Transform3d RIGHT_ROBOT_TO_CAMERA =
        new Transform3d(
            new Translation3d(
                0.0889,   
            -0.2794,   
                0.4445 
            ),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(20), 
                Units.degreesToRadians(-90) 
            )
        );

    public Vision(VisionIO io){
        this.io = io;
    }

    public static Vision getInstance() {
        if (instance == null) {
            switch (Constants.currentMode) {
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
