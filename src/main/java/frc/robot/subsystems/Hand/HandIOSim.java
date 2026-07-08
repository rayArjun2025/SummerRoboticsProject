package frc.robot.subsystems.hand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class HandIOSim implements HandIO {
  private double handVoltage = 0;

  private LinearSystem<N2, N1, N2> handSystem;
  private final PIDController pid;
  private final DCMotorSim handMotorSim;

  public HandIOSim() {
    handSystem = LinearSystemId.createDCMotorSystem(HandConstants.kV, HandConstants.kA);
    pid = new PIDController(HandConstants.kP, HandConstants.kI, HandConstants.kD);
    handMotorSim = new DCMotorSim(handSystem, DCMotor.getKrakenX60Foc(1));
  }

  @Override
  public void updateInputs(HandIOInputs inputs) {
    handMotorSim.update(Constants.globalDelta_s);
    double motorRadPerSec = handMotorSim.getAngularVelocityRadPerSec();

    inputs.handMotorCurrent = handMotorSim.getCurrentDrawAmps();
    inputs.handMotorVolts = handVoltage;
    inputs.handPositionDeg = Math.toDegrees(handMotorSim.getAngularPositionRad());
    inputs.handMotorVelocity_dps = Math.toDegrees(motorRadPerSec);
  }

  @Override
  public void setHandVoltage(double volts_V, double ff_V) {
    handVoltage = MathUtil.clamp(volts_V + ff_V, HandConstants.LOW_CLAMP, HandConstants.HIGH_CLAMP);
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

    handVoltage = pid.calculate(handCurrentDeg, position_deg);
    handVoltage = MathUtil.clamp(handVoltage, HandConstants.LOW_CLAMP, HandConstants.HIGH_CLAMP);
    handMotorSim.setInputVoltage(handVoltage);
  }
}
