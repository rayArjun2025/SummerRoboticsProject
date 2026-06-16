package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorSim implements ElevatorIO{
    private double elevatorPosition = 0;
    private double elevatorVelocity = 0;
    private double elevatorCurrent = 0;
    private double motorVoltage = 0;
    private PIDController pid;

    private LinearSystem<N2, N1, N2> elevatorSystem;
    private final DCMotorSim elevatorMotorSim;
    
    public ElevatorSim(){
        elevatorSystem = LinearSystemId.createDCMotorSystem(1.0, 1.0);
        elevatorMotorSim = new DCMotorSim(elevatorSystem, DCMotor.getKrakenX60Foc(1), 1.0, 1.0);
        pid = new PIDController(1.0, 0.0, 0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        elevatorMotorSim.update(ElevatorConstants.CHANGE_IN_TIME);
        double motorRadiansPerSec = elevatorMotorSim.getAngularVelocityRadPerSec();
        elevatorVelocity = motorRadiansPerSec * ElevatorConstants.DRUM_RADIUS;
        elevatorPosition += elevatorVelocity * ElevatorConstants.CHANGE_IN_TIME;
        elevatorCurrent = elevatorMotorSim.getCurrentDrawAmps();


        inputs.elevatorPositionMeters = MathUtil.clamp(elevatorPosition, ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT);
        inputs.elevatorMotor_rps = motorRadiansPerSec / (2 * Math.PI);
        inputs.elevatorVelocityMetersPerSec = elevatorVelocity;
        inputs.elevatorMotorVolts = motorVoltage;
        inputs.elevatorMotorCurrent = elevatorCurrent;
        inputs.atTop = elevatorPosition >= ElevatorConstants.ELEVATOR_MAX_HEIGHT;
        inputs.atBottom = elevatorPosition <= ElevatorConstants.ELEVATOR_MIN_HEIGHT;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        elevatorMotorSim.setInputVoltage(voltage);
    }

    @Override
    public void setMoveVelocity(double velocity_mps) {
        motorVoltage = pid.calculate(elevatorVelocity, velocity_mps);
        elevatorMotorSim.setInputVoltage(motorVoltage);
    }

    @Override
    public void stopMoving() {
        setMoveVelocity(0);
        setMotorVoltage(0);
    }
    
}
