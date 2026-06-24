// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
    private double motorVoltage = 0;

    private LinearSystem<N2, N1, N2> climberSystem;
    private final DCMotorSim hookMotorSim;
    private final DCMotorSim wheelMotorSim;

    public ClimberIOSim() {
        climberSystem = LinearSystemId.createDCMotorSystem(1.0, 1.0);
        hookMotorSim = new DCMotorSim(climberSystem, DCMotor.getKrakenX60Foc(1));
        wheelMotorSim = new DCMotorSim(climberSystem, DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        hookMotorSim.update(0.02);
        double hookMotorRadPerSec = hookMotorSim.getAngularVelocityRadPerSec();

        inputs.hookOutputCurrent = hookMotorSim.getCurrentDrawAmps();
        inputs.hookOutputVoltage = motorVoltage;
        inputs.hookPositionDeg = Math.toDegrees(hookMotorSim.getAngularPositionRad());
        inputs.hookVelocity = Math.toDegrees(hookMotorRadPerSec);

        wheelMotorSim.update(0.02);
        double wheelMotorRadPerSec = wheelMotorSim.getAngularVelocityRadPerSec();

        inputs.wheelOutputCurrent = wheelMotorSim.getCurrentDrawAmps();
        inputs.wheelOutputVoltage = motorVoltage;
        inputs.wheelPositionDeg = Math.toDegrees(wheelMotorSim.getAngularPositionRad());
        inputs.wheelVelocity = Math.toDegrees(wheelMotorRadPerSec);
    }

    @Override
    public void setHookVoltage(double volts_V, double ff_V) {
        motorVoltage = volts_V + ff_V;
        hookMotorSim.setInputVoltage(motorVoltage);
    }

    @Override
    public void setHookVelocity(double velocity_rps) {
    }

    @Override
    public void stopClimb() {
        setHookVoltage(0.0, 0.0);
    }

    @Override
    public void climbTo(double hook_position_deg, double wheel_position_deg) {
    }
}