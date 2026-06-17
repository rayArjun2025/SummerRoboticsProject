package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorSim implements ElevatorIO{
    private double motorVoltage = 0;

    private LinearSystem<N2, N1, N2> elevatorSystem;
    private final DCMotorSim elevatorMotorSim;
    
    public ElevatorSim(){
        elevatorSystem = LinearSystemId.createDCMotorSystem(1.0, 1.0);
        elevatorMotorSim = new DCMotorSim(elevatorSystem, DCMotor.getKrakenX60Foc(1), ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        elevatorMotorSim.update(ElevatorConstants.CHANGE_IN_TIME);
        double motorRadPerSec = elevatorMotorSim.getAngularVelocityRadPerSec();

        inputs.elevatorMotor_rps = motorRadPerSec / (2 * Math.PI);
        inputs.elevatorVelocityMetersPerSec = motorRadPerSec * ElevatorConstants.DRUM_RADIUS;
        inputs.elevatorMotorVolts = motorVoltage;
        inputs.elevatorPositionMeters = elevatorMotorSim.getAngularPositionRad() * ElevatorConstants.DRUM_RADIUS;
        inputs.elevatorMotorCurrent = elevatorMotorSim.getCurrentDrawAmps();
        
        inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        inputs.atBottom = inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        motorVoltage = voltage;
        elevatorMotorSim.setInputVoltage(voltage);
    }

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }
    
}
