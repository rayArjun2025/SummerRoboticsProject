// Raymond: lowercase package.
package frc.robot.subsystems.Hand;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandIOSim implements HandIO {
    private double motorVoltage = 0;

    private LinearSystem<N2, N1, N2> handSystem;
    private final DCMotorSim handMotorSim;

    public HandIOSim() {
        handSystem = LinearSystemId.createDCMotorSystem(1.0, 1.0);
        handMotorSim = new DCMotorSim(handSystem, DCMotor.getKrakenX60Foc(1));
    }

    @Override
    public void updateInputs(HandIOInputs inputs) {
        handMotorSim.update(0.02);
        double motorRadPerSec = handMotorSim.getAngularVelocityRadPerSec();

        inputs.handMotorCurrent = handMotorSim.getCurrentDrawAmps();
        inputs.handMotorVolts = motorVoltage;
        inputs.handPositionDeg = Math.toDegrees(handMotorSim.getAngularPositionRad());
        inputs.handMotorVelocity = Math.toDegrees(motorRadPerSec);
    }

    @Override
    public void setHandVoltage(double volts_V, double ff_V) {
        motorVoltage = volts_V + ff_V;
        handMotorSim.setInputVoltage(motorVoltage);
    }

    @Override
    public void setHandVelocity(double velocity_rps) {
    }

    @Override
    public void stopMoving() {
        setHandVoltage(0.0, 0.0);
    }

    // Raymond: grip() is empty in sim, so the hand position never changes when you command a grip. that means GRIPPING_CORAL/ALGAE in Hand.java can never hit their target in sim - you can't test the state machine. drive the sim position toward the commanded position like the real one does.
    @Override
    public void grip(double position_deg) {
    }
}