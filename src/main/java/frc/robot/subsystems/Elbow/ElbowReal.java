// Raymond: lowercase package - frc.robot.subsystems.elbow.
package frc.robot.subsystems.Elbow;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

// Raymond: name it ElbowIOReal to match the convention (ClimberIOReal, ServoIOReal) - it's an IO
// impl.
public class ElbowReal implements ElbowIO {

  private final TalonFX elbowMotor;

  private final StatusSignal<Angle> elbowPosition;
  private final StatusSignal<AngularVelocity> elbowVelocity;
  private final StatusSignal<Voltage> elbowVoltage;
  private final StatusSignal<Current> elbowCurrent;

  public ElbowReal() {
    elbowMotor = new TalonFX(ElbowConstants.MOTOR_ID, TunerConstants.kCANBus);

    elbowPosition = elbowMotor.getPosition();
    elbowVelocity = elbowMotor.getVelocity();
    elbowVoltage = elbowMotor.getMotorVoltage();
    elbowCurrent = elbowMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, elbowPosition, elbowVelocity, elbowVoltage, elbowCurrent);

    elbowMotor.optimizeBusUtilization();

    // Raymond: you never actually configure the motor. apply a TalonFXConfiguration with
    // stator/supply current limits (the 60/82 numbers belong in ElbowConstants), neutral mode
    // (brake for an arm), and inverted. without it the motor runs uncapped and unsafe.
  }

  @Override
  public void updateInputs(ElbowIOInputs inputs) {

    var status =
        BaseStatusSignal.refreshAll(elbowPosition, elbowVelocity, elbowVoltage, elbowCurrent);

    inputs.connected = status.isOK();

    inputs.elbowRotateAngle =
        elbowPosition.getValueAsDouble() / ElbowConstants.GEAR_RATIO * 2.0 * Math.PI;

    inputs.angularVelocityRad =
        elbowVelocity.getValueAsDouble() / ElbowConstants.GEAR_RATIO * 2.0 * Math.PI;

    inputs.elbowVoltage = elbowVoltage.getValueAsDouble();

    inputs.elbowCurrent = elbowCurrent.getValueAsDouble();

    inputs.atMaxAngle = inputs.elbowRotateAngle >= ElbowConstants.MAX_ANGLE;

    inputs.atMinAngle = inputs.elbowRotateAngle <= ElbowConstants.MIN_ANGLE;
  }

  @Override
  public void setElbowVoltage(double voltage) {
    // Raymond: clamp to +/-12 (constant) before commanding it.
    elbowMotor.setVoltage(voltage);
  }

  @Override
  public void stopMotor() {
    elbowMotor.stopMotor();
  }
}
