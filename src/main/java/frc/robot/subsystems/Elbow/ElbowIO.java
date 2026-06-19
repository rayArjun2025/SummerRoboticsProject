package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.AutoLog;

public interface ElbowIO {
    @AutoLog
   public static class ElbowIOInputs{
        public double elbowRotateAngle = 0;
        public double angularVelocityRad = 0;
        public double elbowCurrent = 0;
        public double elbowVoltage = 0;
        public boolean atMaxAngle = false;
        public boolean atMinAngle = false;
    }

    public default void updateInputs(ElbowIOInputs inputs){}
    public default void setShoulderVoltage(double volts){}
    public default void stopMotor() {}
}

