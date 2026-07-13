package frc.robot.subsystems.arm;
import org.littletonrobotics.junction.AutoLog;


public interface ArmIO {
    @AutoLog
    public static class ArmIOInputs { // ethan - prefer degrees as these are the values you're
                                    // reading in advantagescope
        public double shoulderSwivelAngle_rad = 0;
        public double elbowSwivelAngle_rad = 0;

        public double elbowAV_rad = 0;
        public double shoulderAV_rad = 0;

        public double shoulderCurrent_amps = 0;
        public double elbowCurrent_amps = 0;

        public double shoulderVoltage_volts = 0;
        public double elbowVoltage_volts = 0;
        public boolean connected = false;
    }

    public default void updateInputs(ArmIOInputs inputs){}
    public default void setElbowVoltage(double volts) {}
    public default void setShoulderVoltage(double volts) {}

    /* ethan - All methods should work for both real and sim. these two don't follow that. very bad */
    public default void swivelElbow() {} // what does this do?
    public default void swivelShoulder() {} // what does this do?

    /* ethan - you will want a method to zero your elbow and shoulder individually */
    public default void setElbowTargetAngle(double angle_rad){}
    public default void setShoulderTargetAngle(double angle_rad) {}

    public default void setShoulderVelocity(double sVelocity_rps){}
    public default void setElbowVelocity(double eVelocity_rps) {}
    
    public default void stopMotor() {}
}
