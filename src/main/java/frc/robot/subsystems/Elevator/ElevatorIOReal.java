package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ElevatorIOReal implements ElevatorIO{
    private final TalonFX elevatorMotor;
    private final StatusSignal<Current> elevatorCurrent;
    private final StatusSignal<Voltage> elevatorVoltage;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final StatusSignal<Angle> elevatorPosition;

    private final VoltageOut elevatorVoltOut_V;

    public ElevatorIOReal(){
        elevatorMotor = new TalonFX(ElevatorConstants.MOTOR_ID, TunerConstants.kCANBus);
        elevatorCurrent = elevatorMotor.getStatorCurrent();
        elevatorVoltage = elevatorMotor.getMotorVoltage();
        elevatorVelocity = elevatorMotor.getVelocity();
        elevatorPosition = elevatorMotor.getPosition();

        var elevatorMotorConfig = new TalonFXConfiguration();

        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        elevatorMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        elevatorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elevatorMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        elevatorMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elevatorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elevatorMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        elevatorMotorConfig.Slot0.kP = ElevatorConstants.KP;
        elevatorMotorConfig.Slot0.kI = ElevatorConstants.KI;
        elevatorMotorConfig.Slot0.kD = ElevatorConstants.KD;
        elevatorMotorConfig.Slot0.kS = ElevatorConstants.KS;
        elevatorMotorConfig.Slot0.kV = ElevatorConstants.KV;
        elevatorMotorConfig.Slot0.kA = ElevatorConstants.KA;

        tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(elevatorMotorConfig));

        elevatorVoltOut_V = new VoltageOut(0).withEnableFOC(true);
        BaseStatusSignal.setUpdateFrequencyForAll(ElevatorConstants.UPDATE_RATE, elevatorCurrent,elevatorVoltage,elevatorVelocity,elevatorPosition);
        elevatorMotor.optimizeBusUtilization();

    }
   
    @Override 
    public void updateInputs(ElevatorIOInputs inputs){
        var status = BaseStatusSignal.refreshAll(elevatorCurrent, elevatorVoltage, elevatorVelocity, elevatorPosition);
        inputs.connected = status.isOK();
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
        elevatorMotor.setControl(elevatorVoltOut_V.withOutput(voltage));
    }

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }
}
