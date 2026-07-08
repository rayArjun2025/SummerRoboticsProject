package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO{
    private SingleJointedArmSim shoulderJoint;
    private SingleJointedArmSim elbowJoint;

    private double shoulderTarget_RAD = 0;
    private double elbowTarget_RAD = 0;
    private double shoulderMotorVoltage = 0;
    private double elbowMotorVoltage = 0;

    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    public ArmIOSim(){
        double shoulderMoi = SingleJointedArmSim.estimateMOI(ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.SHOULDER_ARM_MASS);
        shoulderJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ArmConstants.SHOULDER_GEAR_RATIO, shoulderMoi, ArmConstants.SHOULDER_ARM_LENGTH, ArmConstants.SHOULDER_MIN_ANGLE, ArmConstants.SHOULDER_MAX_ANGLE, true, ArmConstants.SHOULDER_ZERO_REF);

        double elbowMoi = SingleJointedArmSim.estimateMOI(ArmConstants.ELBOW_ARM_LENGTH, ArmConstants.ELBOW_ARM_MASS);
        elbowJoint = new SingleJointedArmSim(DCMotor.getKrakenX60Foc(1), ArmConstants.ELBOW_GEAR_RATIO, elbowMoi, ArmConstants.ELBOW_ARM_LENGTH, ArmConstants.ELBOW_MIN_ANGLE, ArmConstants.ELBOW_MAX_ANGLE, true, ArmConstants.ELBOW_ZERO_REF);

        shoulderPID = new PIDController(ArmConstants.SHOULDER_KP, ArmConstants.SHOULDER_KI, ArmConstants.SHOULDER_KD);
        elbowPID = new PIDController(ArmConstants.ELBOW_KP, ArmConstants.ELBOW_KI, ArmConstants.ELBOW_KD);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs){
        shoulderJoint.update(Constants.globalDelta_s);
        elbowJoint.update(Constants.globalDelta_s);

        inputs.connected = true;

        inputs.shoulderAV_rad = shoulderJoint.getVelocityRadPerSec();
        inputs.shoulderSwivelAngle_rad = shoulderJoint.getAngleRads();
        inputs.shoulderCurrent_amps = shoulderJoint.getCurrentDrawAmps();
        inputs.shoulderVoltage_volts = shoulderMotorVoltage;

        inputs.elbowAV_rad = elbowJoint.getVelocityRadPerSec();
        inputs.elbowSwivelAngle_rad = elbowJoint.getAngleRads();
        inputs.elbowCurrent_amps = elbowJoint.getCurrentDrawAmps();
        inputs.elbowVoltage_volts = elbowMotorVoltage;
    }

   
    @Override
    public void setShoulderTargetAngle(double angle_rad){
        shoulderTarget_RAD = angle_rad;
    }

    @Override
    public void setElbowTargetAngle(double angle_rad){
        elbowTarget_RAD = angle_rad;
    }

    @Override 
    public void swivelElbow(){
        double currentAngle_rad = elbowJoint.getAngleRads();
        double pidOut = elbowPID.calculate(currentAngle_rad, elbowTarget_RAD);
        double ff = ArmConstants.GRAVITY_FF * Math.cos(currentAngle_rad);
        double voltage = MathUtil.clamp(pidOut + ff, ArmConstants.LOW_CLAMP, ArmConstants.HIGH_CLAMP);
        setElbowVoltage(voltage);
    }

    @Override
    public void swivelShoulder(){
        double currentAngle_rad = shoulderJoint.getAngleRads();
        double pidOut = shoulderPID.calculate(currentAngle_rad, shoulderTarget_RAD);
        double ff = ArmConstants.GRAVITY_FF * Math.cos(currentAngle_rad);
        double voltage = MathUtil.clamp(pidOut + ff, ArmConstants.LOW_CLAMP, ArmConstants.HIGH_CLAMP);
        setShoulderVoltage(voltage);
    }

    @Override
    public void setShoulderVoltage(double volts){
        shoulderMotorVoltage = volts;
        shoulderJoint.setInputVoltage(shoulderMotorVoltage);
    }

    @Override
    public void setElbowVoltage(double volts) {
        elbowMotorVoltage = volts;
        elbowJoint.setInputVoltage(elbowMotorVoltage);
    }

    @Override
    public void setShoulderVelocity(double velocity_rps) {}

    @Override
    public void setElbowVelocity(double velocity_rps) {}

    @Override
    public void stopMotor(){
        setShoulderVoltage(0);
        setElbowVoltage(0);
    }
}
