// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: lowercase package.
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
    private final TalonFX wheelMotor;
    // Raymond: these are the HOOK motor's signals but you named them climbXxx. confusing when wheelXxx sits right below it - call them hookCurrent_A/hookVolts_V/etc so it lines up with the motor.
    // Raymond: also these should be private. nothing outside this class needs the raw status signals - reference keeps them private.
    public final StatusSignal<Current> climbCurrent_A;
    public final StatusSignal<Voltage> climbVolts_V;
    public final StatusSignal<AngularVelocity> climbVel_rps;
    public final StatusSignal<Angle> climbPos_r;

    private final VoltageOut climbVoltOut_V;
    private final VelocityVoltage climbVelOut;
    private final PositionVoltage climbPosCtrl;


    public final StatusSignal<Current> wheelCurrent_A;
    public final StatusSignal<Voltage> wheelVolts_V;
    public final StatusSignal<AngularVelocity> wheelVel_rps;
    public final StatusSignal<Angle> wheelPos_r;

    private final VoltageOut wheelVoltOut_V;
    private final VelocityVoltage wheelVelOut;
    private final PositionVoltage wheelPosCtrl;

    // Raymond: brace goes on the same line as the signature (K&R) like the rest of the codebase, not dropped to the next line.
    public ClimberIOReal()
    {
        hookMotor = new TalonFX(ClimberConstants.hookMotorID, TunerConstants.kCANBus);

        climbCurrent_A = hookMotor.getStatorCurrent();
        climbVolts_V = hookMotor.getMotorVoltage();
        climbVel_rps = hookMotor.getVelocity();
        climbPos_r = hookMotor.getPosition();


        //Hook Motor
        var hookMotorConfig = new TalonFXConfiguration();

        // Raymond: 60/82 are copy-pasted here AND in the wheel config below. pull them into ClimberConstants (SUPPLY_CURRENT_LIMIT_A, STATOR_CURRENT_LIMIT_A) so both motors read the same value and you change it in one place.
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

        //Wheel Motor
        wheelMotor = new TalonFX(ClimberConstants.wheelMotorID, TunerConstants.kCANBus);

        wheelCurrent_A = wheelMotor.getStatorCurrent();
        wheelVolts_V = wheelMotor.getMotorVoltage();
        wheelVel_rps = wheelMotor.getVelocity();
        wheelPos_r = wheelMotor.getPosition();

        var wheelMotorConfig = new TalonFXConfiguration();

        wheelMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        wheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        wheelMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        wheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        wheelMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        wheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        wheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        wheelMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
        wheelMotorConfig.Slot0.kP = ClimberConstants.wheelKP;
        wheelMotorConfig.Slot0.kI = ClimberConstants.wheelKI;
        wheelMotorConfig.Slot0.kD = ClimberConstants.wheelKD;
        wheelMotorConfig.Slot0.kS = ClimberConstants.wheelKS;
        wheelMotorConfig.Slot0.kV = ClimberConstants.wheelKV;
        wheelMotorConfig.Slot0.kA = ClimberConstants.wheelKA;

        tryUntilOk(5, () -> wheelMotor.getConfigurator().apply(wheelMotorConfig));

        wheelVoltOut_V = new VoltageOut(0).withEnableFOC(true);
        wheelVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
        wheelPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        BaseStatusSignal.refreshAll(
                climbCurrent_A,
                climbVolts_V,
                climbVel_rps,
                climbPos_r,
                
                wheelCurrent_A,
                wheelVolts_V,
                wheelVel_rps,
                wheelPos_r);

        inputs.hookOutputCurrent = climbCurrent_A.getValueAsDouble();
        inputs.hookOutputVoltage = climbVolts_V.getValueAsDouble();
        inputs.hookPositionDeg = climbPos_r.getValueAsDouble() * 360.0;
        inputs.hookVelocity = climbVel_rps.getValueAsDouble() * 360.0;

        inputs.wheelOutputCurrent = wheelCurrent_A.getValueAsDouble();
        inputs.wheelOutputVoltage = wheelVolts_V.getValueAsDouble();
        inputs.wheelPositionDeg = wheelPos_r.getValueAsDouble() * 360.0;
        inputs.wheelVelocity = wheelVel_rps.getValueAsDouble() * 360.0;
    }

    @Override
    public void setHookVoltage(double volts_V, double ff_V) {
        // Raymond: -12.0 and 12 (no .0) - keep it consistent, both should be 12.0. small thing but it reads sloppy. 12 should really be a named constant too.
        volts_V = MathUtil.clamp(volts_V + ff_V, -12.0, 12);
        hookMotor.setControl(climbVoltOut_V.withOutput(volts_V));
    }

    @Override
    public void setHookVelocity(double velocity_rps) {
        hookMotor.setControl(climbVelOut.withVelocity(velocity_rps));
    }

    @Override
    public void setWheelVoltage(double volts_V, double ff_V) {
        volts_V = MathUtil.clamp(volts_V + ff_V, -12.0, 12);
        wheelMotor.setControl(wheelVoltOut_V.withOutput(volts_V));
    }

    @Override
    public void setWheelVelocity(double velocity_rps) {
        wheelMotor.setControl(wheelVelOut.withVelocity(velocity_rps));
    }

    @Override
    public void stopClimb() {
        setHookVoltage(0.0, 0.0);
        setWheelVoltage(0.0, 0.0);
    }

    @Override
    public void climbTo(double hook_position_deg, double wheel_position_deg) {
        double hook_pos_r = hook_position_deg / 360.0;
        double wheel_pos_r = wheel_position_deg / 360.0;
        hookMotor.setControl(climbPosCtrl.withPosition(hook_pos_r));
        wheelMotor.setControl(wheelPosCtrl.withPosition(wheel_pos_r));
    }

    // Raymond: you never override toggleBrake() or putUpTheWheels() here, so the interface defaults (do nothing) run. if the climber needs a brake that's a real gap, not just style. either implement it or cut it from the interface.
    // Raymond: also no zeroPosition - the climber never gets a known zero, so position control is relative to wherever it powered on. reference homes/zeros on startup.
}