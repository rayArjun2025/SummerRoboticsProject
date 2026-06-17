package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;


public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs{
        public double elevatorMotor_rps = 0.0;
        public double elevatorMotorVolts = 0.0;
        public double elevatorMotorCurrent = 0.0;
        public double elevatorPositionMeters = 0.0;
        public double elevatorVelocityMetersPerSec = 0.0;
        public boolean atTop = false;
        public boolean atBottom = false;
    }


    public default void updateInputs(ElevatorIOInputs inputs) {
        
    }
    public default void setMotorVoltage(double volts) {}
    public default void stopMoving() {}
}
