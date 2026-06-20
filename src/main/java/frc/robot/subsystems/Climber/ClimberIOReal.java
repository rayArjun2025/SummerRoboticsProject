// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.MathUtil;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class ClimberIOReal implements ClimberIO {
    private final TalonFX hookMotor;

    public final StatusSignal<Current> climbCurrent_A;
    public final StatusSignal<Voltage> climbVolts_V;
    public final StatusSignal<AngularVelocity> climbVel_rps;
    public final StatusSignal<Angle> climbPos_r;

    private final VoltageOut climbVoltOut_V;
    private final VelocityVoltage climbVelOut;
    private final PositionVoltage climbPosCtrl;

    public ClimberIOReal()
    {
        hookMotor = new TalonFX(ClimberConstants.hookMotorID, TunerConstants.kCANBus);

        climbCurrent_A = hookMotor.getStatorCurrent();
        climbVolts_V = hookMotor.getMotorVoltage();
        climbVel_rps = hookMotor.getVelocity();
        climbPos_r = hookMotor.getPosition();

        var hookMotorConfig = new TalonFXConfiguration();

        hookMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        hookMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        hookMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        hookMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        hookMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        hookMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        hookMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        hookMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        hookMotorConfig.Slot0.kP = ClimberConstants.hookKP;
        hookMotorConfig.Slot0.kI = ClimberConstants.hookKI;
        hookMotorConfig.Slot0.kD = ClimberConstants.hookKD;
        hookMotorConfig.Slot0.kS = ClimberConstants.hookKS;
        hookMotorConfig.Slot0.kV = ClimberConstants.hookKV;
        hookMotorConfig.Slot0.kA = ClimberConstants.hookKA;

        tryUntilOk(5, () -> hookMotor.getConfigurator().apply(hookMotorConfig));

        climbVoltOut_V = new VoltageOut(0).withEnableFOC(true);
        climbVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
        climbPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                climbCurrent_A,
                climbVolts_V,
                climbVel_rps,
                climbPos_r);

        inputs.hookOutputCurrent = climbCurrent_A.getValueAsDouble();
        inputs.hookOutputVoltage = climbVolts_V.getValueAsDouble();
        inputs.hookPosition = climbPos_r.getValueAsDouble() * 360.0;
        inputs.hookVelocity = climbVel_rps.getValueAsDouble() * 360.0;
    }

    @Override
    public void setHookVoltage(double volts_V, double ff_V) {
        volts_V = MathUtil.clamp(volts_V + ff_V, -12.0, 12);
        hookMotor.setControl(climbVoltOut_V.withOutput(volts_V));
    }

    @Override
    public void setHookVelocity(double velocity_rps) {
        // Onboard TalonFX closed-loop — runs at the motor controller's update
        // rate, not the 50Hz robot loop, so it tracks load disturbances faster.
        hookMotor.setControl(climbVelOut.withVelocity(velocity_rps));
    }

    @Override
    public void stopClimb() {
        setHookVoltage(0.0, 0.0);
    }

    @Override
    public void climbTo(double position_deg) {
        double pos_r = position_deg / 360.0;
        hookMotor.setControl(climbPosCtrl.withPosition(pos_r));
    }
}