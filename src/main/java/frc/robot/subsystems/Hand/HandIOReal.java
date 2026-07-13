// Raymond: lowercase package. and this file is nearly identical to ClimberIOReal - same current
// limits, same FOC setup, same boilerplate. fine for now, but at some point we factor the shared
// TalonFX setup into a helper instead of copy-pasting it into every IOReal.
package frc.robot.subsystems.hand;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class HandIOReal implements HandIO {
  private final TalonFX handMotor;

  private final StatusSignal<Current> handCurrent_A;
  private final StatusSignal<Voltage> handVolts_V;
  private final StatusSignal<AngularVelocity> handVel_rps;
  private final StatusSignal<Angle> handPos_r;

  private final VoltageOut handVoltOut_V;
  private final VelocityVoltage handVelOut;
  private final PositionVoltage handPosCtrl;

  public HandIOReal() {
    handMotor = new TalonFX(HandConstants.motorID, TunerConstants.kCANBus);

    handCurrent_A = handMotor.getStatorCurrent();
    handVolts_V = handMotor.getMotorVoltage();
    handVel_rps = handMotor.getVelocity();
    handPos_r = handMotor.getPosition();

    var handMotorConfig = new TalonFXConfiguration();

    handMotorConfig.CurrentLimits.SupplyCurrentLimit = HandConstants.SUPPLY_CURRENT_LIMIT_A;
    handMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    handMotorConfig.CurrentLimits.StatorCurrentLimit = HandConstants.STATOR_CURRENT_LIMIT_A;
    handMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    handMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
    handMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    handMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    /* ethan - i think pid is less important here? unless we reach a case where we need to run
     * the rollers at a set velocity for controlled grabbing/dropping it's kinda optional. not
     * bad to have though.
     */
    handMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.02;
    handMotorConfig.Slot0.kP = HandConstants.kP;
    handMotorConfig.Slot0.kI = HandConstants.kI;
    handMotorConfig.Slot0.kD = HandConstants.kD;
    handMotorConfig.Slot0.kS = HandConstants.kS;
    handMotorConfig.Slot0.kV = HandConstants.kV;
    handMotorConfig.Slot0.kA = HandConstants.kA;

    tryUntilOk(5, () -> handMotor.getConfigurator().apply(handMotorConfig));

    handVoltOut_V = new VoltageOut(0).withEnableFOC(true);
    handVelOut = new VelocityVoltage(0.0).withEnableFOC(true);
    handPosCtrl = new PositionVoltage(0.0).withEnableFOC(true);
  }

  @Override
  public void updateInputs(HandIOInputs inputs) {
    var status = BaseStatusSignal.refreshAll(handCurrent_A, handVolts_V, handVel_rps, handPos_r);
    inputs.connected = status.isOK();
    inputs.handMotorCurrent = handCurrent_A.getValueAsDouble();
    inputs.handMotorVolts = handVolts_V.getValueAsDouble();
    inputs.handPositionDeg = handPos_r.getValueAsDouble() * 360.0;
    inputs.handMotorVelocity_dps = handVel_rps.getValueAsDouble() * 360.0;
  }

  @Override
  public void setHandVoltage(double volts_V, double ff_V) { // ethan -  just say volts, no ff.
    volts_V = MathUtil.clamp(volts_V + ff_V, HandConstants.LOW_CLAMP, HandConstants.HIGH_CLAMP);
    handMotor.setControl(handVoltOut_V.withOutput(volts_V));
  }

  @Override
  public void setHandVelocity(double velocity_rps) {
    handMotor.setControl(handVelOut.withVelocity(velocity_rps));
  }

  @Override
  public void stopMoving() {
    setHandVoltage(0.0, 0.0);
  }

  @Override
  public void grip(double position_deg) { // ethan - ig that's one way to only grab up coral.
                                          // it might slip.
    double pos_r = position_deg / 360.0;
    handMotor.setControl(handPosCtrl.withPosition(pos_r));
  }

  @Override
  public void zeroPosition() {
    handMotor.setPosition(0.0);
  }
}
