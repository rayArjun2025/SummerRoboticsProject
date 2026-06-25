// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Raymond: lowercase package.
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

        // Raymond: 0.02 is the loop period - it's used in both updates, make it a constant (Constants.loopPeriodSecs) instead of hardcoding the magic number twice.
        wheelMotorSim.update(0.02);
        double wheelMotorRadPerSec = wheelMotorSim.getAngularVelocityRadPerSec();

        inputs.wheelOutputCurrent = wheelMotorSim.getCurrentDrawAmps();
        // Raymond: bug - you report the HOOK's voltage as the wheel's voltage. there's only one motorVoltage field shared between both motors, so setting the hook voltage also changes what the wheel reports. you need a separate voltage var per motor.
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
        // Raymond: this only stops the hook. the wheel keeps whatever voltage it had. stop both.
        setHookVoltage(0.0, 0.0);
    }

    // Raymond: big gap - the sim only drives the hook. you never override setWheelVoltage/setWheelVelocity, and climbTo is empty, so in sim the wheel never moves and position control does nothing. that means the SHALLOW_CLIMBING state in Climber.java can never reach its target in sim and you can't actually test the climber logic. mirror what you did for the hook.
    @Override
    public void climbTo(double hook_position_deg, double wheel_position_deg) {
    }
}