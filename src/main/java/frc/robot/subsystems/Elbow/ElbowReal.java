package frc.robot.subsystems.Elbow;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.generated.TunerConstants;

public class ElbowReal implements ElbowIO {

    private final TalonFX elbowMotor;
    private final CANcoder elbowEncoder;

    private final StatusSignal<Angle> encoderPosition;
    private final StatusSignal<AngularVelocity> motorVelocity;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Current> motorCurrent;

    public ElbowReal() {
        elbowMotor = new TalonFX(
            ElbowConstants.MOTOR_ID,
            TunerConstants.kCANBus
        );

        elbowEncoder = new CANcoder(
            ElbowConstants.CANCODER_ID,
            TunerConstants.kCANBus
        );

        encoderPosition = elbowEncoder.getAbsolutePosition();

        motorVelocity = elbowMotor.getVelocity();
        motorVoltage = elbowMotor.getMotorVoltage();
        motorCurrent = elbowMotor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            encoderPosition,
            motorVelocity,
            motorVoltage,
            motorCurrent
        );
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs) {

        BaseStatusSignal.refreshAll(
            encoderPosition,
            motorVelocity,
            motorVoltage,
            motorCurrent
        );

        inputs.elbowRotateAngle = getElbowAngleRadians();

        inputs.angularVelocityRad =
            motorVelocity.getValueAsDouble()
            / ElbowConstants.GEAR_RATIO
            * 2.0 * Math.PI;

        inputs.elbowCurrent =
            motorCurrent.getValueAsDouble();

        inputs.elbowVoltage =
            motorVoltage.getValueAsDouble();

        inputs.atMaxAngle =
            inputs.elbowRotateAngle >= ElbowConstants.MAX_ANGLE;

        inputs.atMinAngle =
            inputs.elbowRotateAngle <= ElbowConstants.MIN_ANGLE;
    }

    private double getElbowAngleRadians() {
        double rotations = encoderPosition.getValueAsDouble();
        return (rotations - ElbowConstants.CANCODER_OFFSET) * 2.0 * Math.PI;
    }

    @Override
    public void setElbowVoltage(double voltage) {
        elbowMotor.setVoltage(voltage);
    }

    @Override
    public void stopMotor() {
        elbowMotor.stopMotor();
    }
}