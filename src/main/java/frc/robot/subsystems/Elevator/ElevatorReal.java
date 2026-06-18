package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorReal implements ElevatorIO{
    private final TalonFX elevatorMotor = new TalonFX(0);
   
}
