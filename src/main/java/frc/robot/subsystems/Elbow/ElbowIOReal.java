package frc.robot.subsystems.elbow;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ElbowIOReal implements ElbowIO {

    private final TalonFX elbowMotor;

    private final MotionMagicConfigs motionMagicConfigs;
    private final MotionMagicVoltage voltageControl = new MotionMagicVoltage(0);
    private final MotionMagicVelocityVoltage velocityControl = new MotionMagicVelocityVoltage(0);

    private final StatusSignal<Angle> elbowPosition;
    private final StatusSignal<AngularVelocity> elbowVelocity;
    private final StatusSignal<Voltage> elbowVoltage;
    private final StatusSignal<Current> elbowCurrent;

    private double targetAngle_RAD = 0;

    public ElbowIOReal() {
        elbowMotor = new TalonFX(
            ElbowConstants.MOTOR_ID,
            TunerConstants.kCANBus
        );

        elbowPosition = elbowMotor.getPosition();
        elbowVelocity = elbowMotor.getVelocity();
        elbowVoltage = elbowMotor.getMotorVoltage();
        elbowCurrent = elbowMotor.getStatorCurrent();

        var elbowMotorConfig = new TalonFXConfiguration();

        elbowMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        elbowMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        elbowMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        elbowMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        elbowMotorConfig.Feedback.SensorToMechanismRatio = ElbowConstants.GEAR_RATIO;
        elbowMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elbowMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        elbowMotorConfig.Slot0.kP = ElbowConstants.KP;
        elbowMotorConfig.Slot0.kI = ElbowConstants.KI;
        elbowMotorConfig.Slot0.kD = ElbowConstants.KD;
        elbowMotorConfig.Slot0.kS = ElbowConstants.KS;
        elbowMotorConfig.Slot0.kV = ElbowConstants.KV;
        elbowMotorConfig.Slot0.kA = ElbowConstants.KA;
        elbowMotorConfig.Slot0.kG = ElbowConstants.KG;
        elbowMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        motionMagicConfigs = elbowMotorConfig.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = ElbowConstants.MOTION_MAGIC_ACCELERATION;
        motionMagicConfigs.MotionMagicCruiseVelocity = ElbowConstants.MOTION_MAGIC_CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicJerk = ElbowConstants.MOTION_MAGIC_JERK;

        tryUntilOk(5, () -> elbowMotor.getConfigurator().apply(elbowMotorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
            ElbowConstants.UPDATE_RATE,
            elbowPosition,
            elbowVelocity,
            elbowVoltage,
            elbowCurrent
        );

        elbowMotor.optimizeBusUtilization();

        
        
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs) {

        var status = BaseStatusSignal.refreshAll(
            elbowPosition,
            elbowVelocity,
            elbowVoltage,
            elbowCurrent
        );

        inputs.connected = status.isOK();

        inputs.elbowRotateAngleRad =
            elbowPosition.getValueAsDouble()
            * 2.0 * Math.PI;

        inputs.angularVelocityRad =
            elbowVelocity.getValueAsDouble()
            * 2.0 * Math.PI;

        inputs.elbowVoltageVolts =
            elbowVoltage.getValueAsDouble();

        inputs.elbowCurrentAmps =
            elbowCurrent.getValueAsDouble();
    }


    @Override
    public void swivelAngle() {
        double rotations = targetAngle_RAD / (2.0 * Math.PI);
        elbowMotor.setControl(voltageControl.withPosition(rotations));
    }

    @Override
    public void setTargetAngle(double targetAngle_RAD) {
        targetAngle_RAD = MathUtil.clamp(targetAngle_RAD,ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE);
        this.targetAngle_RAD = targetAngle_RAD;
    }

    @Override
    public void setElbowVelocity(double velocity_rad_per_sec) {
        double rotations_per_sec = velocity_rad_per_sec / (2.0 * Math.PI);
        elbowMotor.setControl(velocityControl.withVelocity(rotations_per_sec));
    }

    @Override
    public void stopMotor() {
        elbowMotor.stopMotor();
    }
}