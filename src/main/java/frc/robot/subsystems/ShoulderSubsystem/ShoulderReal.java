package frc.robot.subsystems.ShoulderSubsystem;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShoulderReal implements ShoulderIO {
    private final TalonFX motor;

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<Voltage> voltage;
    private final StatusSignal<Current> current;

    public ShoulderReal() {
        motor = new TalonFX(ShoulderConstants.MOTOR_ID);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            position,
            velocity,
            voltage,
            current
        );

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs) {
        BaseStatusSignal.refreshAll(
            position,
            velocity,
            voltage,
            current
        );

        inputs.shoulderSwivelAngle =
            rotationsToRadians(position.getValueAsDouble());

        inputs.angularVelocityRad =
            rotationsToRadians(velocity.getValueAsDouble());

        inputs.shoulderVoltage =
            voltage.getValueAsDouble();

        inputs.shoulderCurrent =
            current.getValueAsDouble();
        
        inputs.atMaxAngle = inputs.shoulderSwivelAngle >= ShoulderConstants.ZERO_REF + ShoulderConstants.MAX_ANGLE;
         inputs.atMinAngle = inputs.shoulderSwivelAngle <= ShoulderConstants.ZERO_REF + ShoulderConstants.MIN_ANGLE;
    }
    
    public static double rotationsToRadians(double motorRotations) {
        double armRotations = motorRotations / ShoulderConstants.GEAR_RATIO;
        return armRotations * 2.0 * Math.PI;
    }

    @Override
    public void setShoulderVoltage(double volts) {
        motor.setVoltage(volts);
    }
}