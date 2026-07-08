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
  private final TalonFX climberMotor;

  private final StatusSignal<Current> climberCurrent_A;
  private final StatusSignal<Voltage> climberVolts_V;
  private final StatusSignal<AngularVelocity> climberVel_rps;
  private final StatusSignal<Angle> climberPos_r;

  private final VoltageOut climberVoltOut_V;
  private final VelocityVoltage climberVelOut;
  private final PositionVoltage climberPosCtrl;

 

  public ClimberIOReal() {

    climberMotor = new TalonFX(ClimberConstants.hookMotorID, TunerConstants.kCANBus);

    climberCurrent_A = climberMotor.getStatorCurrent();
    climberVolts_V = climberMotor.getMotorVoltage();
    climberVel_rps = climberMotor.getVelocity();
    climberPos_r = climberMotor.getPosition();

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
    hookMotorConfig.Slot0.kP = ClimberConstants.hookKP;
    hookMotorConfig.Slot0.kI = ClimberConstants.hookKI;
    hookMotorConfig.Slot0.kD = ClimberConstants.hookKD;
    hookMotorConfig.Slot0.kS = ClimberConstants.hookKS;
    hookMotorConfig.Slot0.kV = ClimberConstants.hookKV;
    hookMotorConfig.Slot0.kA = ClimberConstants.hookKA;

    tryUntilOk(5, () -> climberMotor.getConfigurator().apply(hookMotorConfig));

    climberVoltOut_V = new VoltageOut(0).withEnableFOC(true);
    climberVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
    climberPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);

   
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(
        climberCurrent_A,
        climberVolts_V,
        climberVel_rps,
        climberPos_r);

    inputs.connected = status.isOK();
    inputs.climberCurrent = climberCurrent_A.getValueAsDouble();
    inputs.climberVoltage = climberVolts_V.getValueAsDouble();
    inputs.climberPositionDeg = climberPos_r.getValueAsDouble() * 360.0;
    inputs.climberVelocity_dps = climberVel_rps.getValueAsDouble() * 360.0;
  }

  @Override
  public void setClimberVoltage(double volts_V, double ff_V) {
    volts_V =
        MathUtil.clamp(volts_V + ff_V, -ClimberConstants.maxVoltage, ClimberConstants.maxVoltage);
    climberMotor.setControl(climberVoltOut_V.withOutput(volts_V));
  }

  @Override
  public void setClimberVelocity(double velocity_rps) {
    climberMotor.setControl(climberVelOut.withVelocity(velocity_rps));
  }


  @Override
  public void stopClimb() {
    setClimberVoltage(0.0, 0.0);
  }

  @Override
  public void climbTo(double climber_pos_deg) {
    double climberPosRad = climber_pos_deg / 360.0;
    climberMotor.setControl(climberPosCtrl.withPosition(climberPosRad));
  }

  public void zeroPosition() {
    climberMotor.setPosition(0.0);
  }

}
