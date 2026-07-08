// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;


import edu.wpi.first.math.system.plant.DCMotor;

import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

import frc.robot.subsystems.elbow.ElbowConstants;

public class ClimberIOSim implements ClimberIO {
  private double climberVoltage = 0.00;

  private SingleJointedArmSim climberSim;

  public ClimberIOSim() {
    double moi = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
    climberSim = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(2), ClimberConstants.GEAR_RATIO, moi, ClimberConstants.ARM_LENGTH, ClimberConstants.homeDegrees_deg, ClimberConstants.targetDegrees_deg, true, 0);
  }

  @Override
  public void updateInputs(ClimberIOInputs inputs) {
    inputs.connected = true;

  }

  @Override
  public void setClimberVoltage(double volts_V, double ff_V) {
    climberVoltage = volts_V + ff_V;
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
    
    climberVoltage =
        Math.max(
            -12.0, Math.min(12.0, ClimberConstants.hookKP * (climber_pos_deg - climberCurrentDeg)));

    climberSim.setInputVoltage(climberVoltage);
  }
}
