package frc.robot.subsystems.shoulder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ShoulderReal implements ShoulderIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    private final MotionMagicConfigs motionMagicConfigs;
    private final MotionMagicVoltage voltageControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

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

        shoulderMotorConfig.Feedback.SensorToMechanismRatio = ShoulderConstants.GEAR_RATIO;
        shoulderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shoulderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        shoulderMotorConfig.Slot0.kP = ShoulderConstants.KP;
        shoulderMotorConfig.Slot0.kI = ShoulderConstants.KI;
        shoulderMotorConfig.Slot0.kD = ShoulderConstants.KD;
        shoulderMotorConfig.Slot0.kS = ShoulderConstants.KS;
        shoulderMotorConfig.Slot0.kV = ShoulderConstants.KV;
        shoulderMotorConfig.Slot0.kA = ShoulderConstants.KA;
        shoulderMotorConfig.Slot0.kG = ShoulderConstants.KG;
        shoulderMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motionMagicConfigs = shoulderMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ShoulderConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicCruiseVelocity = ShoulderConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicJerk = ShoulderConstants.MOTION_MAGIC_JERK;

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
        
        
    }
    
    public static double rotationsToRadians(double motorRotations) {
        return motorRotations * 2.0 * Math.PI;
    }

    @Override
    public void setTargetAngle(double targetAngle_RAD) {
        double rotations = targetAngle_RAD / (2.0 * Math.PI);
        motor.setControl(voltageControl.withPosition(rotations));
    }
    

    @Override
    public void setShoulderVelocity(double velocity_rad_per_sec) {
        double rotations_per_sec = velocity_rad_per_sec / (2.0 * Math.PI);
        motor.setControl(velocityControl.withVelocity(rotations_per_sec));
    }

    @Override
    public void stopMotor(){
        motor.setVoltage(0);
    }
}