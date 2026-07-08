package frc.robot.subsystems.hand;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HandIOSim implements HandIO {
  private double handVoltage = 0;

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
    inputs.handMotorVolts = handVoltage;
    inputs.handPositionDeg = Math.toDegrees(handMotorSim.getAngularPositionRad());
    inputs.handMotorVelocity_dps = Math.toDegrees(motorRadPerSec);
  }

  @Override
  public void setHandVoltage(double volts_V, double ff_V) {
    handVoltage = volts_V + ff_V;
    handMotorSim.setInputVoltage(handVoltage);
  }

  @Override
  public void setHandVelocity(double velocity_rps) {}

  @Override
  public void stopMoving() {
    setHandVoltage(0.0, 0.0);
  }

  @Override
  public void grip(double position_deg) {
    double handCurrentDeg = Math.toDegrees(handMotorSim.getAngularPositionRad());

    handVoltage =
        Math.max(
            -HandConstants.maxVoltage,
            Math.min(HandConstants.maxVoltage, HandConstants.kP * (position_deg - handCurrentDeg)));

    handMotorSim.setInputVoltage(handVoltage);
  }
}
