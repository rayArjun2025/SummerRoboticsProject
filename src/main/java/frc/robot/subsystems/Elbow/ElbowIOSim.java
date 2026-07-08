package frc.robot.subsystems.elbow;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ElbowIOSim implements ElbowIO{
    private final SingleJointedArmSim elbowJoint;
    private double motorVoltage;
    private final double MOI;
    private final PIDController pid = new PIDController(ElbowConstants.KP, ElbowConstants.KI, ElbowConstants.KD);
    private double targetAngle_RAD = 0;

    public ElbowIOSim(){
        MOI = SingleJointedArmSim.estimateMOI(ElbowConstants.ARM_LENGTH, ElbowConstants.ARM_MASS);
        elbowJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ElbowConstants.GEAR_RATIO, MOI, ElbowConstants.ARM_LENGTH, ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE, true, ElbowConstants.ZERO_REF);
    }

    @Override
    public void updateInputs(ElbowIOInputs inputs){
        swivelAngle();
        elbowJoint.update(Constants.globalDelta_s);
        inputs.angularVelocityRad = elbowJoint.getVelocityRadPerSec();
        inputs.elbowRotateAngleRad = elbowJoint.getAngleRads();
        inputs.elbowCurrentAmps = elbowJoint.getCurrentDrawAmps();
        inputs.elbowVoltageVolts = motorVoltage;
        inputs.connected = true;
    }

    public void setElbowVoltage(double voltage){
        motorVoltage = voltage;
        elbowJoint.setInputVoltage(voltage);    
    }

    @Override
    public void swivelAngle(){
        double currentAngle = elbowJoint.getAngleRads();
        double ff = ElbowConstants.GRAVITY_FF * Math.cos(currentAngle);
        double pidOut = pid.calculate(currentAngle, targetAngle_RAD);
        double volts = MathUtil.clamp(pidOut + ff, ElbowConstants.MIN_VOLTAGE, ElbowConstants.MAX_VOLTAGE);
        setElbowVoltage(volts);
    }

    @Override
    public void setTargetAngle(double targetAngle_RAD){
        targetAngle_RAD = MathUtil.clamp(targetAngle_RAD,ElbowConstants.MIN_ANGLE, ElbowConstants.MAX_ANGLE);
        this.targetAngle_RAD = targetAngle_RAD;
    }

    @Override
    public void stopMotor(){
        elbowJoint.setInputVoltage(0);
    }
}