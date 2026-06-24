package frc.robot.subsystems.Hand;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.MathUtil;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

public class HandIOReal implements HandIO {
    private final TalonFX handMotor;

    public final StatusSignal<Current> handCurrent_A;
    public final StatusSignal<Voltage> handVolts_V;
    public final StatusSignal<AngularVelocity> handVel_rps;
    public final StatusSignal<Angle> handPos_r;

    private final VoltageOut handVoltOut_V;
    private final VelocityVoltage handVelOut;
    private final PositionVoltage handPosCtrl;

    public HandIOReal()
    {
        handMotor = new TalonFX(HandConstants.motorID, TunerConstants.kCANBus);

        handCurrent_A = handMotor.getStatorCurrent();
        handVolts_V = handMotor.getMotorVoltage();
        handVel_rps = handMotor.getVelocity();
        handPos_r = handMotor.getPosition();

        var handMotorConfig = new TalonFXConfiguration();

        handMotorConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
        handMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        handMotorConfig.CurrentLimits.StatorCurrentLimit = 82.0;
        handMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        handMotorConfig.Feedback.SensorToMechanismRatio = 1.0;
        handMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        handMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

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
        BaseStatusSignal.refreshAll(
                handCurrent_A,
                handVolts_V,
                handVel_rps,
                handPos_r);

        inputs.handMotorCurrent = handCurrent_A.getValueAsDouble();
        inputs.handMotorVolts = handVolts_V.getValueAsDouble();
        inputs.handPositionDeg = handPos_r.getValueAsDouble() * 360.0;
        inputs.handMotorVelocity = handVel_rps.getValueAsDouble() * 360.0;
    }

    @Override
    public void setHandVoltage(double volts_V, double ff_V) {
        volts_V = MathUtil.clamp(volts_V + ff_V, -12.0, 12);
        handMotor.setControl(handVoltOut_V.withOutput(volts_V));
    }

    @Override
    public void setHandVelocity(double velocity_rps) {
        // Onboard TalonFX closed-loop — runs at the motor controller's update
        // rate, not the 50Hz robot loop, so it tracks load disturbances faster.
        handMotor.setControl(handVelOut.withVelocity(velocity_rps));
    }

    @Override
    public void stopMoving() {
        setHandVoltage(0.0, 0.0);
    }

    @Override
    public void grip(double position_deg) {
        double pos_r = position_deg / 360.0;
        handMotor.setControl(handPosCtrl.withPosition(pos_r));
    }
}