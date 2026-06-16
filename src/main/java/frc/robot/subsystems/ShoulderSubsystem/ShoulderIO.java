package frc.robot.subsystems.ShoulderSubsystem;
import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.DigitalInput;

public interface ShoulderIO {
   public static class ShoulderIOInputs{
        public double shoulderPosition = 0;

        
    }

    public default void updateInputs(ShoulderIOInputs inputs){}
    
}
