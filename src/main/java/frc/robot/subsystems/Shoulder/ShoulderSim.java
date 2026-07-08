// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
package frc.robot.subsystems.Shoulder;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderSim implements ShoulderIO {
  private SingleJointedArmSim shoulderJoint;
  private double motorVoltage;

  public ShoulderSim() {
    // Raymond: spotlessApply, this line and the next run way past the margin.
    double moi =
        SingleJointedArmSim.estimateMOI(ShoulderConstants.ARM_LENGTH, ShoulderConstants.ARM_MASS);
    shoulderJoint =
        new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ShoulderConstants.GEAR_RATIO,
            moi,
            ShoulderConstants.ARM_LENGTH,
            ShoulderConstants.MIN_ANGLE,
            ShoulderConstants.MAX_ANGLE,
            true,
            ShoulderConstants.ZERO_REF);
  }

  @Override
  public void updateInputs(ShoulderIOInputs inputs) {
    shoulderJoint.update(ShoulderConstants.CHANGE_IN_TIME);

    inputs.angularVelocityRad = shoulderJoint.getVelocityRadPerSec();
    inputs.shoulderSwivelAngle = shoulderJoint.getAngleRads();
    inputs.shoulderCurrent = shoulderJoint.getCurrentDrawAmps();
    inputs.shoulderVoltage = motorVoltage;

    inputs.atMaxAngle =
        inputs.shoulderSwivelAngle >= ShoulderConstants.MAX_ANGLE + ShoulderConstants.ZERO_REF;
    inputs.atMinAngle =
        inputs.shoulderSwivelAngle <= ShoulderConstants.MIN_ANGLE + ShoulderConstants.ZERO_REF;
  }

  @Override
  public void setShoulderVoltage(double voltage) {
    // Raymond: clamp to +/-12 before setInputVoltage, otherwise sim applies whatever the PID spits
    // out and won't behave like the real robot which is capped at battery voltage.
    motorVoltage = voltage;
    shoulderJoint.setInputVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    shoulderJoint.setInputVoltage(0);
  }
}
