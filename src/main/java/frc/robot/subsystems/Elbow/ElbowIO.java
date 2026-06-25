// Raymond: lowercase package - frc.robot.subsystems.elbow.
package frc.robot.subsystems.Elbow;

import org.littletonrobotics.junction.AutoLog;

public interface ElbowIO {
    // Raymond: indentation is off here (3 spaces vs 4) and the brace has no leading space - spotlessApply.
    @AutoLog
   public static class ElbowIOInputs{
        // Raymond: put units in these names like the rest of the repo (_rad, _radps, _A, _V). this one is radians so elbowRotateAngle_rad.
        public double elbowRotateAngle = ElbowConstants.ZERO_REF;
        // Raymond: this is angular velocity in rad/s but "Rad" reads like a position. name it angularVelocity_radps.
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

