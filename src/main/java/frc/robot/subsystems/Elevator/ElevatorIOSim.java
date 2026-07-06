package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO{
    private double motorVoltage = 0;
    private final ElevatorSim elevatorSim;
    private double targetPosition_m = 0;
    private final PIDController pid;
    
    public ElevatorIOSim(){
       elevatorSim = new ElevatorSim(DCMotor.getKrakenX60Foc(1), ElevatorConstants.GEAR_RATIO, ElevatorConstants.CARRIAGE_MASS, ElevatorConstants.DRUM_RADIUS, ElevatorConstants.ELEVATOR_MIN_HEIGHT, ElevatorConstants.ELEVATOR_MAX_HEIGHT,true,ElevatorConstants.ELEVATOR_MIN_HEIGHT);
       pid = new PIDController(ElevatorConstants.KP, ElevatorConstants.KI, ElevatorConstants.KD);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {

        elevatorSim.update(Constants.globalDelta_s);

        inputs.elevatorVelocityMetersPerSec = elevatorSim.getVelocityMetersPerSecond();
        inputs.elevatorMotorVolts = motorVoltage;
        inputs.elevatorPositionMeters = elevatorSim.getPositionMeters();
        inputs.elevatorMotorCurrent = elevatorSim.getCurrentDrawAmps();
        
       
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

    @Override
    public void moveElevator(){
        double currentPosition = elevatorSim.getPositionMeters();
        double ff = ElevatorConstants.GRAVITY_FF;
        double pidOut = pid.calculate(currentPosition, targetPosition_m);
        double volts = MathUtil.clamp(pidOut + ff, ElevatorConstants.LOW_CLAMP, ElevatorConstants.HIGH_CLAMP);
        setMotorVoltage(volts);
    }
    
    @Override
    public void setTargetPosition(double targetPosition_m) {
        this.targetPosition_m = targetPosition_m;
    }
}
