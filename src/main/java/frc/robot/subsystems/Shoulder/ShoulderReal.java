// Raymond: capitalized package - frc.robot.subsystems.shoulder, rename the folder.
package frc.robot.subsystems.Shoulder;

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
        // Raymond: no TalonFXConfiguration anywhere in here - you never set stator/supply current limits (60 supply / 82 stator). a Kraken with no current limit will brown out the bus or cook itself.
        motor = new TalonFX(ShoulderConstants.MOTOR_ID);

        position = motor.getPosition();
        velocity = motor.getVelocity();
        voltage = motor.getMotorVoltage();
        current = motor.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(
            // Raymond: 50 is a magic number, put the update rate in ShoulderConstants.
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
        var status = BaseStatusSignal.refreshAll(
            position,
            velocity,
            voltage,
            current
        );

        inputs.connected = status.isOK();

        inputs.shoulderSwivelAngle =
            rotationsToRadians(position.getValueAsDouble());

        inputs.angularVelocityRad =
            rotationsToRadians(velocity.getValueAsDouble());

        inputs.shoulderVoltage =
            voltage.getValueAsDouble();

        inputs.shoulderCurrent =
            current.getValueAsDouble();
        
        // Raymond: spotlessApply - the next line is indented one space off. bigger issue: this uses ZERO_REF + MAX/MIN, but the real encoder position starts at 0 here while ShoulderSim starts at ZERO_REF. the two IOs don't agree on where zero is, so atMax/atMin fire at different real angles between sim and real. nail down what zero means and offset it once.
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
    // Raymond: you never override stopMotor() here, so on the real robot it falls through to the empty default in the interface and does nothing - the motor won't actually stop. implement it (motor.setVoltage(0) or a NeutralOut). sim has it, real doesn't.
}