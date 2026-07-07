// Raymond: lowercase package.
package frc.robot.subsystems.hand;

import org.littletonrobotics.junction.AutoLog;


public interface HandIO {
    @AutoLog
    public static class HandIOInputs{ // Raymond: space before the brace - HandIOInputs {
        public double handMotorVolts = 0.0;
        public double handMotorCurrent = 0.0;
        public double handPositionDeg = 0.0;
        public double handMotorVelocity = 0.0; // Raymond: unit suffix - _dps. you convert to degrees in the IO.
        // Raymond: fullClose/fullOpen are populated by neither IOReal nor IOSim - they're always false. either read the real sensors (limit switches?) or remove them. and "full" booleans usually read better as a sensor name like coralDetected/limitSwitchPressed.
        public boolean fullClose = false;
        public boolean fullOpen = false;
    }

    public default void updateInputs(HandIOInputs inputs) {}

    public default void setHandVoltage(double volts_V, double ff_V) {}

    public default void setHandVelocity(double velocity_rps) {}

    public default void stopMoving() {}

    public default void grip(double position) {}
}
