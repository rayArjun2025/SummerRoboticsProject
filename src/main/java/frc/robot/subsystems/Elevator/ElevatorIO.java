package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;



public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double elevatorMotorVolts = 0.0;
        public double elevatorMotorCurrent = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMetersPerSec = 0.0;
        public boolean connected = false;
    }


    public default void updateInputs(ElevatorIOInputs inputs) {}
    public default void setMotorVoltage(double volts) {}
    public default void moveElevator() {}
    public default void stopMoving() {}
    public default void setTargetPosition(double targetPosition_m) {}
    public default void setMotorVelocity(double velocity) {}
}
