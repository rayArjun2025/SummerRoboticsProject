package frc.robot.subsystems.Elbow;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ElbowSim implements ElbowIO{
    private SingleJointedArmSim shoulderJoint;
    private double motorVoltage;

    public ElbowSim(){
        double moi = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
        shoulderJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ElbowConstants.GEAR_RATIO, moi, ElbowConstants.ARM_LENGTH, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE, true, ElbowConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs){
        shoulderJoint.update(ElbowConstants.CHANGE_IN_TIME);

        inputs.angularVelocityRad = shoulderJoint.getVelocityRadPerSec();
        inputs.elbowRotateAngle = shoulderJoint.getAngleRads();
        inputs.elbowCurrent = shoulderJoint.getCurrentDrawAmps();
        inputs.elbowVoltage = motorVoltage;

        inputs.atMaxAngle = inputs.elbowRotateAngle >= ElbowConstants.MAX_ANGLE + ElbowConstants.ZERO_REF;
        inputs.atMinAngle = inputs.elbowRotateAngle <= ElbowConstants.MIN_ANGLE + ElbowConstants.ZERO_REF;
    }

    @Override
    public void setShoulderVoltage(double voltage){
        motorVoltage = voltage;
        shoulderJoint.setInputVoltage(voltage);
    }

    @Override
    public void stopMotor(){
        shoulderJoint.setInputVoltage(0);
    }
}