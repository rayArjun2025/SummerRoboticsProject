package frc.robot.subsystems.elevator;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO{
    private double motorVoltage = 0;
    private final ElevatorSim elevatorSim;
    
    public ElevatorIOSim(){
       elevatorSim = new ElevatorSim(DCMotor.getKrakenX60Foc(1), ElevatorConstants.GEAR_RATIO, ElevatorConstants.CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS, ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT,true,ElevatorConstants.ELEVATOR_MIN_HEIGHT);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        elevatorSim.update(Constants.globalDelta_s);

        inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorMotorVolts = motorVoltage;
        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        inputs.elevatorMotorCurrent = elevatorSim.getCurrentDrawAmps();
        
        inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT - ElevatorConstants.TOLERANCE_METERS;
        inputs.atBottom = inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT + ElevatorConstants.TOLERANCE_METERS;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motorVoltage = voltage;
        elevatorSim.setInputVoltage(motorVoltage);
    }

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }
    
}
