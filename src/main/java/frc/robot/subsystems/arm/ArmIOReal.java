package frc.robot.subsystems.arm;

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
import frc.robot.generated.TunerConstants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ArmIOReal implements ArmIO {
    private final TalonFX shoulderMotor;
    private final TalonFX elbowMotor;

    private final StatusSignal<Angle> shoulderPosition;
    private final StatusSignal<AngularVelocity> shoulderVelocity;
    private final StatusSignal<Voltage> shoulderVoltage;
    private final StatusSignal<Current> shoulderCurrent;

    private final StatusSignal<Angle> elbowPosition;
    private final StatusSignal<AngularVelocity> elbowVelocity;
    private final StatusSignal<Voltage> elbowVoltage;
    private final StatusSignal<Current> elbowCurrent;

    private final MotionMagicConfigs shoulderMotionMagicConfigs;
    private final MotionMagicVoltage shoulderVoltageControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage shoulderVelocityControl = new MotionMagicVelocityVoltage(0);

    private final MotionMagicConfigs elbowMotionMagicConfigs;
    private final MotionMagicVoltage elbowVoltageControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage elbowVelocityControl = new MotionMagicVelocityVoltage(0);


    public ArmIOReal() {
        shoulderMotor = new TalonFX(ArmConstants.SHOULDER_MOTOR_ID, TunerConstants.kCANBus);
        elbowMotor = new TalonFX(ArmConstants.ELBOW_MOTOR_ID, TunerConstants.kCANBus);

        shoulderPosition = shoulderMotor.getPosition();
        shoulderVelocity = shoulderMotor.getVelocity();
        shoulderVoltage = shoulderMotor.getMotorVoltage();
        shoulderCurrent = shoulderMotor.getStatorCurrent();

        elbowPosition = elbowMotor.getPosition();
        elbowVelocity = elbowMotor.getVelocity();
        elbowVoltage = elbowMotor.getMotorVoltage();
        elbowCurrent = elbowMotor.getStatorCurrent();

        var shoulderMotorConfig = new TalonFXConfiguration();

        shoulderMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        shoulderMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        shoulderMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        shoulderMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        shoulderMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.SHOULDER_GEAR_RATIO;
        shoulderMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        shoulderMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        shoulderMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        shoulderMotorConfig.Slot0.kP = ArmConstants.SHOULDER_KP;
        shoulderMotorConfig.Slot0.kI = ArmConstants.SHOULDER_KI;
        shoulderMotorConfig.Slot0.kD = ArmConstants.SHOULDER_KD;
        shoulderMotorConfig.Slot0.kS = ArmConstants.SHOULDER_KS;
        shoulderMotorConfig.Slot0.kV = ArmConstants.SHOULDER_KV;
        shoulderMotorConfig.Slot0.kA = ArmConstants.SHOULDER_KA;
        shoulderMotorConfig.Slot0.kG = ArmConstants.ARM_KG;
        shoulderMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        shoulderMotionMagicConfigs = shoulderMotorConfig.MotionMagic;
        shoulderMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.SHOULDER_MOTION_MAGIC_ACCELERATION;
        shoulderMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.SHOULDER_MOTION_MAGIC_CRUISE_VELOCITY;
        shoulderMotionMagicConfigs.MotionMagicJerk = ArmConstants.SHOULDER_MOTION_MAGIC_JERK;

        tryUntilOk(5, () -> shoulderMotor.getConfigurator().apply(shoulderMotorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(ArmConstants.UPDATE_RATE,shoulderPosition,shoulderVelocity,shoulderVoltage,shoulderCurrent);
        shoulderMotor.optimizeBusUtilization();

        var elbowMotorConfig = new TalonFXConfiguration();
        
        elbowMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        elbowMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        
        elbowMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        elbowMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        elbowMotorConfig.Feedback.SensorToMechanismRatio = ArmConstants.ELBOW_GEAR_RATIO;
        elbowMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elbowMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        elbowMotorConfig.Slot0.kP = ArmConstants.ELBOW_KP;
        elbowMotorConfig.Slot0.kI = ArmConstants.ELBOW_KI;
        elbowMotorConfig.Slot0.kD = ArmConstants.ELBOW_KD;
        elbowMotorConfig.Slot0.kS = ArmConstants.ELBOW_KS;
        elbowMotorConfig.Slot0.kV = ArmConstants.ELBOW_KV;
        elbowMotorConfig.Slot0.kA = ArmConstants.ELBOW_KA;
        elbowMotorConfig.Slot0.kG = ArmConstants.ARM_KG;
        elbowMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        elbowMotionMagicConfigs = elbowMotorConfig.MotionMagic;
        elbowMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.ELBOW_MOTION_MAGIC_ACCELERATION;
        elbowMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.ELBOW_MOTION_MAGIC_CRUISE_VELOCITY;
        elbowMotionMagicConfigs.MotionMagicJerk = ArmConstants.ELBOW_MOTION_MAGIC_JERK;
        BaseStatusSignal.setUpdateFrequencyForAll(ArmConstants.UPDATE_RATE,elbowPosition,elbowVelocity,elbowVoltage,elbowCurrent);
        elbowMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        var status = BaseStatusSignal.refreshAll(
            shoulderPosition,
            shoulderCurrent,
            shoulderVoltage,
            shoulderCurrent,
            elbowPosition,
            elbowCurrent,
            elbowVoltage,
            elbowCurrent
        );

        inputs.connected = status.isOK();

        inputs.shoulderSwivelAngle_rad = rotationsToRadians(shoulderPosition.getValueAsDouble());
        inputs.shoulderAV_rad = rotationsToRadians(shoulderVelocity.getValueAsDouble());
        inputs.shoulderVoltage_volts =shoulderVoltage.getValueAsDouble();
        inputs.shoulderCurrent_amps =shoulderCurrent.getValueAsDouble();

        inputs.elbowSwivelAngle_rad = rotationsToRadians(elbowPosition.getValueAsDouble());
        inputs.elbowAV_rad = rotationsToRadians(elbowVelocity.getValueAsDouble());
        inputs.elbowVoltage_volts = elbowVoltage.getValueAsDouble();
        inputs.elbowCurrent_amps = elbowCurrent.getValueAsDouble();
        
        
    }
    
    public static double rotationsToRadians(double motorRotations) {
        return motorRotations * 2.0 * Math.PI;
    }

    @Override
    public void setShoulderTargetAngle(double targetAngle_RAD) {
        double rotations = targetAngle_RAD / (2.0 * Math.PI);
        shoulderMotor.setControl(shoulderVoltageControl.withPosition(rotations));
    }

    @Override
    public void setElbowTargetAngle(double targetAngle_RAD) {
        double rotations = targetAngle_RAD / (2.0 * Math.PI);
        elbowMotor.setControl(elbowVoltageControl.withPosition(rotations));
    }
    

    @Override
    public void setShoulderVelocity(double velocity_rad_per_sec) {
        double rotations_per_sec = velocity_rad_per_sec / (2.0 * Math.PI);
        shoulderMotor.setControl(shoulderVelocityControl.withVelocity(rotations_per_sec));
    }

     @Override
    public void setElbowVelocity(double velocity_rad_per_sec) {
        double rotations_per_sec = velocity_rad_per_sec / (2.0 * Math.PI);
        elbowMotor.setControl(elbowVelocityControl.withVelocity(rotations_per_sec));
    }

    @Override
    public void stopMotor(){
        shoulderMotor.setVoltage(0);
        elbowMotor.setVoltage(0);
    }
}