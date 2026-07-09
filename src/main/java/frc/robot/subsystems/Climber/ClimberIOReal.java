package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
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

public class ClimberIOReal implements ClimberIO {

    private final TalonFX hookMotor;
    private final TalonFX wheelMotor;

    private double targetRotations;

    private final StatusSignal<Current> hookCurrent_A;
    private final StatusSignal<Voltage> hookVolts_V;
    private final StatusSignal<AngularVelocity> hookVel_rps;
    private final StatusSignal<Angle> hookPos_r;

    private final StatusSignal<Current> wheelCurrent_A;
    private final StatusSignal<Voltage> wheelVolts_V;
    private final StatusSignal<AngularVelocity> wheelVel_rps;
    private final StatusSignal<Angle> wheelPos_r;

    private final VoltageOut hookVoltOut_V;
    private final VelocityVoltage hookVelOut;
    private final PositionVoltage hookPosCtrl;

    private final VoltageOut wheelVoltOut_V;
    private final VelocityVoltage wheelVelOut;
    private final PositionVoltage wheelPosCtrl;

    public ClimberIOReal() {

        hookMotor = new TalonFX(ClimberConstants.hookMotorID, TunerConstants.kCANBus);
        wheelMotor = new TalonFX(ClimberConstants.wheelMotorID, TunerConstants.kCANBus);

        hookCurrent_A = hookMotor.getStatorCurrent();
        hookVolts_V = hookMotor.getMotorVoltage();
        hookVel_rps = hookMotor.getVelocity();
        hookPos_r = hookMotor.getPosition();

        wheelCurrent_A = wheelMotor.getStatorCurrent();
        wheelVolts_V = wheelMotor.getMotorVoltage();
        wheelVel_rps = wheelMotor.getVelocity();
        wheelPos_r = wheelMotor.getPosition();

        var hookConfig = new TalonFXConfiguration();

        hookConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT_A;
        hookConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hookConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT_A;
        hookConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hookConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

        hookConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hookConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hookConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        hookConfig.Slot0.kP = ClimberConstants.climberKP;
        hookConfig.Slot0.kI = ClimberConstants.climberKI;
        hookConfig.Slot0.kD = ClimberConstants.climberKD;
        hookConfig.Slot0.kS = ClimberConstants.climberKS;
        hookConfig.Slot0.kG = ClimberConstants.climberKG;
        hookConfig.Slot0.kV = ClimberConstants.climberKV;
        hookConfig.Slot0.kA = ClimberConstants.climberKA;
        hookConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        tryUntilOk(5, () -> hookMotor.getConfigurator().apply(hookConfig));

        hookVoltOut_V = new VoltageOut(0).withEnableFOC(true);
        hookVelOut = new VelocityVoltage(0).withEnableFOC(true);
        hookPosCtrl = new PositionVoltage(0).withEnableFOC(true);

        var wheelConfig = new TalonFXConfiguration();

        wheelConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT_A;
        wheelConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT_A;
        wheelConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wheelConfig.Feedback.SensorToMechanismRatio = ClimberConstants.GEAR_RATIO;

        wheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wheelConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;

        wheelConfig.Slot0.kP = ClimberConstants.climberKP;
        wheelConfig.Slot0.kI = ClimberConstants.climberKI;
        wheelConfig.Slot0.kD = ClimberConstants.climberKD;
        wheelConfig.Slot0.kS = ClimberConstants.climberKS;
        wheelConfig.Slot0.kG = ClimberConstants.climberKG;
        wheelConfig.Slot0.kV = ClimberConstants.climberKV;
        wheelConfig.Slot0.kA = ClimberConstants.climberKA;
        wheelConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        tryUntilOk(5, () -> wheelMotor.getConfigurator().apply(wheelConfig));

        wheelVoltOut_V = new VoltageOut(0).withEnableFOC(true);
        wheelVelOut = new VelocityVoltage(0).withEnableFOC(true);
        wheelPosCtrl = new PositionVoltage(0).withEnableFOC(true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {

        var status = BaseStatusSignal.refreshAll(
            hookCurrent_A,
            hookVolts_V,
            hookVel_rps,
            hookPos_r,
            wheelCurrent_A,
            wheelVolts_V,
            wheelVel_rps,
            wheelPos_r);

        inputs.connected = status.isOK();
        inputs.climberCurrent = hookCurrent_A.getValueAsDouble() + wheelCurrent_A.getValueAsDouble();
        inputs.climberVoltage = hookVolts_V.getValueAsDouble();
        inputs.climberPositionDeg = hookPos_r.getValueAsDouble() * 360.0;
        inputs.climberVelocity_dps = hookVel_rps.getValueAsDouble() * 360.0;
    }

    @Override
    public void setClimberVoltage(double volts_V) {
        volts_V = MathUtil.clamp(volts_V, ClimberConstants.LOW_CLAMP, ClimberConstants.HIGH_CLAMP);
        hookMotor.setControl(hookVoltOut_V.withOutput(volts_V));
        wheelMotor.setControl(wheelVoltOut_V.withOutput(volts_V));
    }

    @Override
    public void setClimberVelocity(double velocity_rps) {
        hookMotor.setControl(hookVelOut.withVelocity(velocity_rps));
        wheelMotor.setControl(wheelVelOut.withVelocity(velocity_rps));
    }

    @Override
    public void stopClimb() {
        setClimberVoltage(0);
    }

    @Override
    public void climbTo() {
        hookMotor.setControl(hookPosCtrl.withPosition(targetRotations));
        wheelMotor.setControl(wheelPosCtrl.withPosition(targetRotations));
    }

    @Override
    public void setTargetAngle(double targetAngleDeg) {
        targetAngleDeg = MathUtil.clamp(targetAngleDeg, ClimberConstants.MIN_DEG, ClimberConstants.MAX_DEG);
        targetRotations = targetAngleDeg / 360.0;
        climbTo();
    }

    @Override
    public void zeroPosition() {
        hookMotor.setPosition(0);
        wheelMotor.setPosition(0);
    }
}