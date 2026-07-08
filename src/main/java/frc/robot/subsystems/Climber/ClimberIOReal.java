// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
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

    // Hook Motor
    var hookMotorConfig = new TalonFXConfiguration();

    hookMotorConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT_A;
    hookMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    hookMotorConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT_A;
    hookMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hookMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    hookMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    hookMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    hookMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    hookMotorConfig.Slot0.kP = ClimberConstants.climberKP;
    hookMotorConfig.Slot0.kI = ClimberConstants.climberKI;
    hookMotorConfig.Slot0.kD = ClimberConstants.climberKD;
    hookMotorConfig.Slot0.kS = ClimberConstants.climberKS;
    hookMotorConfig.Slot0.kV = ClimberConstants.climberKV;
    hookMotorConfig.Slot0.kA = ClimberConstants.climberKA;

    tryUntilOk(5, () -> hookMotor.getConfigurator().apply(hookMotorConfig));

    hookVoltOut_V = new VoltageOut(0).withEnableFOC(true);
    hookVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
    hookPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);

    // Wheel Motor
    var wheelMotorConfig = new TalonFXConfiguration();

    wheelMotorConfig.CurrentLimits.SupplyCurrentLimit = ClimberConstants.SUPPLY_CURRENT_LIMIT_A;
    wheelMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    wheelMotorConfig.CurrentLimits.StatorCurrentLimit = ClimberConstants.STATOR_CURRENT_LIMIT_A;
    wheelMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    wheelMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    wheelMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wheelMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    wheelMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    wheelMotorConfig.Slot0.kP = ClimberConstants.climberKP;
    wheelMotorConfig.Slot0.kI = ClimberConstants.climberKI;
    wheelMotorConfig.Slot0.kD = ClimberConstants.climberKD;
    wheelMotorConfig.Slot0.kS = ClimberConstants.climberKS;
    wheelMotorConfig.Slot0.kV = ClimberConstants.climberKV;
    wheelMotorConfig.Slot0.kA = ClimberConstants.climberKA;

    tryUntilOk(5, () -> wheelMotor.getConfigurator().apply(wheelMotorConfig));

    wheelVoltOut_V = new VoltageOut(0).withEnableFOC(true);
    wheelVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
    wheelPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(
        hookCurrent_A,
        hookVolts_V,
        hookVel_rps,
        hookPos_r,
        wheelCurrent_A, wheelVolts_V, wheelPos_r, wheelVel_rps);

    inputs.connected = status.isOK();
    inputs.climberCurrent = hookCurrent_A.getValueAsDouble() + wheelCurrent_A.getValueAsDouble();
    inputs.climberVoltage = hookVolts_V.getValueAsDouble();
    inputs.climberPositionDeg = hookPos_r.getValueAsDouble() * 360.0;
    inputs.climberVelocity_dps = hookVel_rps.getValueAsDouble() * 360.0;
  }

  @Override
  public void setClimberVoltage(double volts_V, double ff_V) {
    volts_V =
        MathUtil.clamp(volts_V + ff_V, -ClimberConstants.maxVoltage, ClimberConstants.maxVoltage);
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
    setClimberVoltage(0.0, 0.0);
  }

  @Override
  public void climbTo(double climber_pos_deg) {
    double climberPosRad = climber_pos_deg / 360.0;
    hookMotor.setControl(hookPosCtrl.withPosition(climberPosRad));
    wheelMotor.setControl(wheelPosCtrl.withPosition(climberPosRad));
  }

  public void zeroPosition() {
    hookMotor.setPosition(0.0);
    wheelMotor.setPosition(0.0);
  }

}
