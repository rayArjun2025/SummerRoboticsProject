package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ElevatorReal implements ElevatorIO{
    private final TalonFX elevatorMotor;
    public final StatusSignal<Current> elevatorCurrent;
    public final StatusSignal<Voltage> elevatorVoltage;
    public final StatusSignal<AngularVelocity> elevatorVelocity;
    public final StatusSignal<Angle> elevatorPosition;

    public ElevatorReal(){
        elevatorMotor = new TalonFX(ElevatorConstants.MOTOR_ID, TunerConstants.kCANBus);
        elevatorCurrent = elevatorMotor.getStatorCurrent();
        elevatorVoltage = elevatorMotor.getMotorVoltage();
        elevatorVelocity = elevatorMotor.getVelocity();
        elevatorPosition = elevatorMotor.getPosition();

        BaseStatusSignal.setUpdateFrequencyForAll(50,elevatorCurrent,elevatorVoltage,elevatorVelocity,elevatorPosition);
        elevatorMotor.optimizeBusUtilization();

    }
   
    @Override 
    public void updateInputs(ElevatorIOInputs inputs){
        BaseStatusSignal.refreshAll(elevatorCurrent, elevatorVoltage, elevatorVelocity, elevatorPosition);
        inputs.connected = BaseStatusSignal.refreshAll(elevatorCurrent,elevatorVoltage,elevatorVelocity,elevatorPosition).isOK();
        inputs.elevatorMotorCurrent = elevatorCurrent.getValueAsDouble();
        inputs.elevatorMotorVolts = elevatorVoltage.getValueAsDouble();
        inputs.elevatorVelocityMetersPerSec = convertToLinearVel(elevatorVelocity.getValueAsDouble());
        inputs.elevatorPositionMeters = convertToMeters(elevatorPosition.getValueAsDouble());
        inputs.atTop = inputs.elevatorPositionMeters >= ElevatorConstants.ELEVATOR_MAX_HEIGHT - 0.001;
        inputs.atBottom = inputs.elevatorPositionMeters <= ElevatorConstants.ELEVATOR_MIN_HEIGHT + 0.001;
    }

    private double convertToLinearVel(double aVelocity){
        return aVelocity / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
    }

    private double convertToMeters(double aPos){
         return aPos / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
    }

    @Override
    public void setMotorVoltage(double voltage) {
        elevatorMotor.setVoltage(voltage);
    }

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }
}
