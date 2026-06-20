package frc.robot.subsystems.Hand;

import org.littletonrobotics.junction.AutoLog;


public interface HandIO {
    @AutoLog
    public static class HandIOInputs{
        public double handMotorVolts = 0.0;
        public double handMotorCurrent = 0.0;
        public boolean fullClose = false;
        public boolean fullOpen = false;
    }


    public default void updateInputs(HandIOInputs inputs) {
        
    }
    public default void setMotorVoltage(double volts) {}
    public default void setMoveVelocity(double velocity)  {}
    public default void stopMoving() {}
}
