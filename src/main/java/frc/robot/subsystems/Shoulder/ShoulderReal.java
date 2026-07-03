package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.elevator.ElevatorConstants;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ShoulderReal implements ShoulderIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    public ShoulderReal() {
        motor = new TalonFX(ShoulderConstants.MOTOR_ID);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getStatorCurrent();

        var shoulderMotorConfig = new TalonFXConfiguration();

        shoulderMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        shoulderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shoulderMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        shoulderMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shoulderMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        shoulderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shoulderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        shoulderMotorConfig.Slot0.kP = ShoulderConstants.KP;
        shoulderMotorConfig.Slot0.kI = ShoulderConstants.KI;
        shoulderMotorConfig.Slot0.kD = ShoulderConstants.KD;
        shoulderMotorConfig.Slot0.kS = ShoulderConstants.KS;
        shoulderMotorConfig.Slot0.kV = ShoulderConstants.KV;
        shoulderMotorConfig.Slot0.kA = ShoulderConstants.KA;

        tryUntilOk(5, () -> motor.getConfigurator().apply(shoulderMotorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
            ShoulderConstants.UPDATE_RATE,
            position,
            velocity,
            voltage,
            current
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(
            position,
            velocity,
            voltage,
            current
        );

        inputs.connected = status.isOK();

        inputs.shoulderSwivelAngle_rad =
            rotationsToRadians(position.getValueAsDouble());

        inputs.angularVelocityRad =
            rotationsToRadians(velocity.getValueAsDouble());

        inputs.shoulderVoltage_volts =
            voltage.getValueAsDouble();

        inputs.shoulderCurrent_amps =
            current.getValueAsDouble();
        
        inputs.atMaxAngleRad = inputs.shoulderSwivelAngle_rad >= ShoulderConstants.ZERO_REF + ShoulderConstants.MAX_ANGLE;
        inputs.atMinAngleRad = inputs.shoulderSwivelAngle_rad <= ShoulderConstants.ZERO_REF + ShoulderConstants.MIN_ANGLE;
    }
    
    public static double rotationsToRadians(double motorRotations) {
        double armRotations = motorRotations / ShoulderConstants.GEAR_RATIO;
        return armRotations * 2.0 * Math.PI;
    }

    @Override
    public void setShoulderVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stopMotor(){
        motor.setVoltage(0);
    }
}