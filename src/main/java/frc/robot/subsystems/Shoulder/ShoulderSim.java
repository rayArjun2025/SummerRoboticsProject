package frc.robot.subsystems.shoulder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ShoulderSim implements ShoulderIO{
    private SingleJointedArmSim shoulderJoint;
    private double targetAngle_RAD;

    public ShoulderSim(){
        double moi = SingleJointedArmSim.estimateMOI(ShoulderConstants.ARM_LENGTH, ShoulderConstants.ARM_MASS);
        shoulderJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ShoulderConstants.GEAR_RATIO, moi, ShoulderConstants.ARM_LENGTH, ShoulderConstants.MIN_ANGLE, ShoulderConstants.MAX_ANGLE, true, ShoulderConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ShoulderIOInputs inputs){
        double error = targetAngle_RAD - shoulderJoint.getAngleRads();
        double motorVoltage = MathUtil.clamp(ShoulderConstants.KP * error, -12.0,12.0);
        shoulderJoint.setInputVoltage(motorVoltage);
        shoulderJoint.update(Constants.globalDelta_s);

        inputs.angularVelocityRad = shoulderJoint.getVelocityRadPerSec();
        inputs.shoulderSwivelAngle_rad = shoulderJoint.getAngleRads();
        inputs.shoulderCurrent_amps = shoulderJoint.getCurrentDrawAmps();
        inputs.shoulderVoltage_volts = motorVoltage;
    }

   
    @Override
    public void setTargetAngle(double angle_rad){
        targetAngle_RAD = angle_rad;
    }

    @Override
    public void stopMotor(){
        shoulderJoint.setInputVoltage(0);
    }
}
