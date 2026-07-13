package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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
    
    private double targetPosition_m = 0;

    private final MotionMagicVoltage voltageControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

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

        var motionMagicConfigs = elevatorMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicJerk = ElevatorConstants.MOTION_MAGIC_JERK;

        tryUntilOk(5, () -> elevatorMotor.getConfigurator().apply(elevatorMotorConfig));

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
    }

    private double convertToLinearVel(double aVelocity){ // ethan - (omega * r)/g = v. not sure
                                                         // where you got the circomference from.
        return aVelocity / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
    }

    private double convertToMeters(double aPos){ //  ethan - same for converting to linear vel.
         return aPos / ElevatorConstants.GEAR_RATIO * 2 * Math.PI * ElevatorConstants.DRUM_RADIUS;
    }

    /* ethan - very misleading name. looks more like moveToPosition() or moveToHeight() */
    @Override
    public void setMotorVoltage(double position) {
        elevatorMotor.setControl(voltageControl.withPosition(position));
    }

    @Override
    public void setMotorVelocity(double velocity) { // ethan - look into VelocityOut
        elevatorMotor.setControl(velocityControl.withVelocity(velocity));
    }

    /* ethan - no actual direct voltage control? look into VoltageOut. */

    @Override
    public void stopMoving() {
        setMotorVoltage(0);
    }

    @Override
    public void moveElevator() { // ethan - pretty unnecessary. you have setMotorVoltage() already.
                                 // the name is also pretty misleading.
        setMotorVoltage(targetPosition_m);
    }

    @Override
    public void setTargetPosition(double targetPosition_m) { // ethan - only motor and sensor
                                                             // control stuff should be handled
                                                             // here; target position and more
                                                             // abstract (?) aspects should be
                                                             // dealt with in Elevator.java
        this.targetPosition_m = targetPosition_m;
    }
}
