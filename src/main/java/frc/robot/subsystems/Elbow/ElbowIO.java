package frc.robot.subsystems.elbow;

import org.littletonrobotics.junction.AutoLog;

public interface ElbowIO {
   @AutoLog
   public class ElbowIOInputs{
        public double elbowRotateAngleRad = ElbowConstants.ZERO_REF;
        public double angularVelocityRad = 0;
        public double elbowCurrentAmps = 0;
        public double elbowVoltageVolts = 0;
        public boolean connected = false;
    }

    public default void updateInputs(ElbowIOInputs inputs){}
    public default void swivelAngle() {}
    public default void stopMotor() {}
    public default void setTargetAngle(double targetAngle_RAD) {}
    public default void setElbowVelocity(double velocity_rad_per_sec) {}
}

