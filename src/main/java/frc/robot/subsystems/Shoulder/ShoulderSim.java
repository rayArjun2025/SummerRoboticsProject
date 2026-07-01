package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ShoulderSim implements ShoulderIO{
    private SingleJointedArmSim shoulderJoint;
    private double motorVoltage;

    public ShoulderSim(){
        double moi = SingleJointedArmSim.estimateMOI(ShoulderConstants.ARM_LENGTH, ShoulderConstants.ARM_MASS);
        shoulderJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ShoulderConstants.GEAR_RATIO, moi, ShoulderConstants.ARM_LENGTH, ShoulderConstants.MIN_ANGLE, ShoulderConstants.MAX_ANGLE, true, ShoulderConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs){
        shoulderJoint.update(ShoulderConstants.CHANGE_IN_TIME);

        inputs.angularVelocityRad = shoulderJoint.getVelocityRadPerSec();
        inputs.shoulderSwivelAngle_rad = shoulderJoint.getAngleRads();
        inputs.shoulderCurrent_amps = shoulderJoint.getCurrentDrawAmps();
        inputs.shoulderVoltage_volts = motorVoltage;

        inputs.atMaxAngleRad = inputs.shoulderSwivelAngle_rad >= ShoulderConstants.MAX_ANGLE + ShoulderConstants.ZERO_REF;
        inputs.atMinAngleRad = inputs.shoulderSwivelAngle_rad <= ShoulderConstants.MIN_ANGLE + ShoulderConstants.ZERO_REF;
    }

    @Override
    public void setShoulderVoltage(double voltage){
        motorVoltage = MathUtil.clamp(voltage, ShoulderConstants.LOW_CLAMP, ShoulderConstants.HIGH_CLAMP);
        shoulderJoint.setInputVoltage(voltage);
    }

    @Override
    public void stopMotor(){
        shoulderJoint.setInputVoltage(0);
    }
}
