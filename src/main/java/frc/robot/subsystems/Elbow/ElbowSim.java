package frc.robot.subsystems.Elbow;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ElbowSim implements ElbowIO{
    private SingleJointedArmSim elbowJoint;
    private double motorVoltage;

    public ElbowSim(){
        double moi = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
        elbowJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ElbowConstants.GEAR_RATIO, moi, ElbowConstants.ARM_LENGTH, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE, true, ElbowConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs){
        elbowJoint.update(ElbowConstants.CHANGE_IN_TIME);

        inputs.angularVelocityRad = elbowJoint.getVelocityRadPerSec();
        inputs.elbowRotateAngle = elbowJoint.getAngleRads();
        inputs.elbowCurrent = elbowJoint.getCurrentDrawAmps();
        inputs.elbowVoltage = motorVoltage;

        inputs.atMaxAngle = inputs.elbowRotateAngle >= ElbowConstants.MAX_ANGLE + ElbowConstants.ZERO_REF;
        inputs.atMinAngle = inputs.elbowRotateAngle <= ElbowConstants.MIN_ANGLE + ElbowConstants.ZERO_REF;
    }

    @Override
    public void setElbowVoltage(double voltage){
        motorVoltage = voltage;
        elbowJoint.setInputVoltage(voltage);
    }

    @Override
    public void stopMotor(){
        elbowJoint.setInputVoltage(0);
    }
}