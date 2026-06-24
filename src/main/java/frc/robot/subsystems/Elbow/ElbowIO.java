package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.AutoLog;

public interface ElbowIO {
    @AutoLog
   public static class ElbowIOInputs{
        public double elbowRotateAngle = ElbowConstants.ZERO_REF;
        public double angularVelocityRad = 0;
        public double elbowCurrent = 0;
        public double elbowVoltage = 0;
        public boolean atMaxAngle = false;
        public boolean atMinAngle = false;
        public boolean connected = false;
    }

    public default void updateInputs(ElbowIOInputs inputs){}
    public default void setElbowVoltage(double volts){}
    public default void stopMotor() {}
}

