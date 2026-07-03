package frc.robot.subsystems.elbow;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
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

    private final StatusSignal<Angle> elbowPosition;
    private final StatusSignal<AngularVelocity> elbowVelocity;
    private final StatusSignal<Voltage> elbowVoltage;
    private final StatusSignal<Current> elbowCurrent;

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

        elbowMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        elbowMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        elbowMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        elbowMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        elbowMotorConfig.Slot0.kP = ElbowConstants.KP;
        elbowMotorConfig.Slot0.kI = ElbowConstants.KI;
        elbowMotorConfig.Slot0.kD = ElbowConstants.KD;
        elbowMotorConfig.Slot0.kS = ElbowConstants.KS;
        elbowMotorConfig.Slot0.kV = ElbowConstants.KV;
        elbowMotorConfig.Slot0.kA = ElbowConstants.KA;

        tryUntilOk(5, () -> elbowMotor.getConfigurator().apply(elbowMotorConfig));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
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
            / ElbowConstants.GEAR_RATIO
            * 2.0 * Math.PI;

        inputs.angularVelocityRad =
            elbowVelocity.getValueAsDouble()
            / ElbowConstants.GEAR_RATIO
            * 2.0 * Math.PI;

        inputs.elbowVoltageVolts =
            elbowVoltage.getValueAsDouble();

        inputs.elbowCurrentAmps =
            elbowCurrent.getValueAsDouble();

        inputs.atMaxAngleRad =
            inputs.elbowRotateAngleRad >= ElbowConstants.MAX_ANGLE;

        inputs.atMinAngleRad =
            inputs.elbowRotateAngleRad <= ElbowConstants.MIN_ANGLE;
    }

    @Override
    public void setElbowVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, ElbowConstants.MIN_VOLTAGE, ElbowConstants.MAX_VOLTAGE);
        elbowMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        elbowMotor.stopMotor();
    }
}