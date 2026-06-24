package frc.robot.subsystems.Hand;

import org.littletonrobotics.junction.AutoLog;


public interface HandIO {
    @AutoLog
    public static class HandIOInputs{
        public double handMotorVolts = 0.0;
        public double handMotorCurrent = 0.0;
        public double handPositionDeg = 0.0;
        public double handMotorVelocity = 0.0;
        public boolean fullClose = false;
        public boolean fullOpen = false;
    }

    public default void updateInputs(HandIOInputs inputs) {}

    public default void setHandVoltage(double volts_V, double ff_V) {}

    public default void setHandVelocity(double velocity_rps) {}

    public default void stopMoving() {}

    public default void grip(double position) {}
}
