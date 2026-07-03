package frc.robot.subsystems.elbow;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ElbowIOSim implements ElbowIO{
    private final SingleJointedArmSim elbowJoint;
    private double motorVoltage;
    private final double MOI;

    public ElbowIOSim(){
        MOI = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
        elbowJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ElbowConstants.GEAR_RATIO, MOI, ElbowConstants.ARM_LENGTH, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE, true, ElbowConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs){
        elbowJoint.update(Constants.globalDelta_s);

        inputs.angularVelocityRad = elbowJoint.getVelocityRadPerSec();
        inputs.elbowRotateAngleRad = elbowJoint.getAngleRads();
        inputs.elbowCurrentAmps = elbowJoint.getCurrentDrawAmps();
        inputs.elbowVoltageVolts = motorVoltage;
        
        inputs.atMaxAngleRad = inputs.elbowRotateAngleRad >= ElbowConstants.MAX_ANGLE;
        inputs.atMinAngleRad = inputs.elbowRotateAngleRad <= ElbowConstants.MIN_ANGLE;
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