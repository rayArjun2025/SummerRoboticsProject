package frc.robot.subsystems.ShoulderSubsystem;
import org.littletonrobotics.junction.AutoLog;


public interface ShoulderIO {

    @AutoLog
   public static class ShoulderIOInputs{
        public double shoulderSwivelAngle = 0;
        public double angularVelocityRad = 0;
        public double shoulderCurrent = 0;
        public double shoulderVoltage = 0;
        public boolean atMaxAngle = false;
        public boolean atMinAngle = false;
    }

    public default void updateInputs(ShoulderIOInputs inputs){}
    public default void setShoulderVoltage(double volts){}
    public default void stopMotor() {}
}
