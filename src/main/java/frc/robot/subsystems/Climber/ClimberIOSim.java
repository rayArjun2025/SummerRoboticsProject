// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
  private double climberVoltage = 0.00;
  private final PIDController pid;
  private SingleJointedArmSim climberSim;

  public ClimberIOSim() {
    pid = new PIDController(ClimberConstants.climberKP, ClimberConstants.climberKI, ClimberConstants.climberKD);
    double moi = SingleJointedArmSim.estimateMOI(ClimberConstants.ARM_LENGTH, ClimberConstants.ARM_MASS);
    climberSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(2), ClimberConstants.GEAR_RATIO, moi, ClimberConstants.ARM_LENGTH, Math.toRadians(ClimberConstants.homeDegrees_deg), Math.toRadians(ClimberConstants.targetDegrees_deg), true, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    climberSim.update(Constants.globalDelta_s);

    inputs.connected = true;
    inputs.climberCurrent = climberSim.getCurrentDrawAmps();
    inputs.climberPositionDeg = Math.toDegrees(climberSim.getAngleRads());
    inputs.climberVelocity_dps = Math.toDegrees(climberSim.getVelocityRadPerSec());
    inputs.climberVoltage = climberVoltage;
  }

  @Override
  public void setClimberVoltage(double volts_V, double ff_V) {
    climberVoltage = volts_V + ff_V;
    climberVoltage = MathUtil.clamp(climberVoltage, ClimberConstants.LOW_CLAMP, ClimberConstants.HIGH_CLAMP);
    climberSim.setInputVoltage(climberVoltage);
  }

  @Override
  public void setClimberVelocity(double velocity_rps) {}

  @Override
  public void stopClimb() {
    setClimberVoltage(0, 0);
  }

  @Override
  public void climbTo(double climber_pos_deg) {
    double climberCurrentDeg = Math.toDegrees(climberSim.getAngleRads());
    climberVoltage = pid.calculate(Math.toRadians(climberCurrentDeg), Math.toRadians(climber_pos_deg));
    climberVoltage = MathUtil.clamp(climberVoltage, ClimberConstants.LOW_CLAMP, ClimberConstants.HIGH_CLAMP);
    climberSim.setInputVoltage(climberVoltage);
  }
}
