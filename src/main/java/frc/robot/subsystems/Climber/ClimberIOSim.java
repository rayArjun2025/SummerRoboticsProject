// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimberIOSim implements ClimberIO {
    private double hookVoltage = 0.00;
    private double wheelVoltage = 0.00;

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
        hookMotorSim.update(ClimberConstants.loopPeriodSecs);
        double hookMotorRadPerSec = hookMotorSim.getAngularVelocityRadPerSec();

        inputs.hookOutputCurrent = hookMotorSim.getCurrentDrawAmps();
        inputs.hookOutputVoltage = hookVoltage;
        inputs.hookPositionDeg = Math.toDegrees(hookMotorSim.getAngularPositionRad());
        inputs.hookVelocity = Math.toDegrees(hookMotorRadPerSec);

        wheelMotorSim.update(ClimberConstants.loopPeriodSecs);
        double wheelMotorRadPerSec = wheelMotorSim.getAngularVelocityRadPerSec();

        inputs.wheelOutputCurrent = wheelMotorSim.getCurrentDrawAmps();
        inputs.wheelOutputVoltage = wheelVoltage;
        inputs.wheelPositionDeg = Math.toDegrees(wheelMotorSim.getAngularPositionRad());
        inputs.wheelVelocity = Math.toDegrees(wheelMotorRadPerSec);
    }

    @Override
    public void setHookVoltage(double volts_V, double ff_V) {
        hookVoltage = volts_V + ff_V;
        hookMotorSim.setInputVoltage(hookVoltage);
    }

    @Override
    public void setHookVelocity(double velocity_rps) {
    }

    @Override
    public void setWheelVoltage(double volts_V, double ff_V) {
        wheelVoltage = volts_V + ff_V;
        wheelMotorSim.setInputVoltage(wheelVoltage);
    }

    @Override
    public void setWheelVelocity(double velocity_rps) {
    }

    @Override
    public void stopClimb() {
        setHookVoltage(0.0, 0.0);
        setWheelVoltage(0.0, 0.0);
    }

    @Override
    public void climbTo(double hook_position_deg, double wheel_position_deg) {
        double hookCurrentDeg  = Math.toDegrees(hookMotorSim.getAngularPositionRad());
        double wheelCurrentDeg = Math.toDegrees(wheelMotorSim.getAngularPositionRad());

        hookVoltage  = Math.max(-12.0, Math.min(12.0, ClimberConstants.hookKP * (hook_position_deg  - hookCurrentDeg)));
        wheelVoltage = Math.max(-12.0, Math.min(12.0, ClimberConstants.wheelKP * (wheel_position_deg - wheelCurrentDeg)));

        hookMotorSim.setInputVoltage(hookVoltage);
        wheelMotorSim.setInputVoltage(wheelVoltage);
    }
}