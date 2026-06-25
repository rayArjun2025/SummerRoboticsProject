// Raymond: lowercase package - frc.robot.subsystems.elbow.
package frc.robot.subsystems.Elbow;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// Raymond: name it ElbowIOSim to match ClimberIOSim/ServoIOSim. and missing space before the brace - spotlessApply.
public class ElbowSim implements ElbowIO{
    // Raymond: these can be final, neither is reassigned.
    private SingleJointedArmSim elbowJoint;
    private double motorVoltage;

    public ElbowSim(){
        double moi = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
        elbowJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ElbowConstants.GEAR_RATIO, moi, ElbowConstants.ARM_LENGTH, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE, true, ElbowConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs){
        // Raymond: use Constants.globalDelta_s for the sim step like everything else, not your own CHANGE_IN_TIME copy of the loop period.
        elbowJoint.update(ElbowConstants.CHANGE_IN_TIME);

        inputs.angularVelocityRad = elbowJoint.getVelocityRadPerSec();
        inputs.elbowRotateAngle = elbowJoint.getAngleRads();
        inputs.elbowCurrent = elbowJoint.getCurrentDrawAmps();
        inputs.elbowVoltage = motorVoltage;

        // Raymond: ZERO_REF is 0, so "+ ZERO_REF" does nothing here - drop it. just compare against MAX_ANGLE / MIN_ANGLE.
        inputs.atMaxAngle = inputs.elbowRotateAngle >= ElbowConstants.MAX_ANGLE + ElbowConstants.ZERO_REF;
        inputs.atMinAngle = inputs.elbowRotateAngle <= ElbowConstants.MIN_ANGLE + ElbowConstants.ZERO_REF;
    }

    @Override
    public void setElbowVoltage(double voltage){
        // Raymond: clamp to +/-12 here too (MathUtil.clamp) so sim matches what the real battery can actually deliver.
        motorVoltage = voltage;
        elbowJoint.setInputVoltage(voltage);
    }

    @Override
    public void stopMotor(){
        elbowJoint.setInputVoltage(0);
    }
}