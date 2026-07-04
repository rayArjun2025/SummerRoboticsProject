package frc.robot.subsystems.shoulder;
import org.littletonrobotics.junction.AutoLog;


public interface ShoulderIO {

    @AutoLog
   public static class ShoulderIOInputs{
        public double shoulderSwivelAngle_rad = 0;
        public double angularVelocityRad = 0;
        public double shoulderCurrent_amps = 0;
        public double shoulderVoltage_volts = 0;
        public boolean connected = false;
    }

    public default void updateInputs(ShoulderIOInputs inputs){}
    public default void setTargetAngle(double angle_rad){}
    public default void setShoulderVelocity(double velocity_rad_per_sec){}
    public default void stopMotor() {}
}
