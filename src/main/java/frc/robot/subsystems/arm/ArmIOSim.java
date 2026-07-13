package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmIOSim implements ArmIO {

    private SingleJointedArmSim shoulderJoint;
    private SingleJointedArmSim elbowJoint;

    private ArmFeedforward elbowFF;
    private ArmFeedforward shoulderFF;

    private TrapezoidProfile shoulderProfile;
    private TrapezoidProfile elbowProfile;

    private TrapezoidProfile.State shoulderGoal;
    private TrapezoidProfile.State shoulderCurrent;

    private TrapezoidProfile.State elbowGoal;
    private TrapezoidProfile.State elbowCurrent;

    private double shoulderMotorVoltage = 0;
    private double elbowMotorVoltage = 0;

    private final PIDController shoulderPID;
    private final PIDController elbowPID;

    public ArmIOSim() {
        elbowProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ArmConstants.ELBOW_MAX_VELOCITY,
                ArmConstants.ELBOW_MAX_ACCELERATION));

        shoulderProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                ArmConstants.SHOULDER_MAX_VELOCITY,
                ArmConstants.SHOULDER_MAX_ACCELERATION));

        elbowGoal = new TrapezoidProfile.State(
            ArmConstants.ELBOW_ZERO_REF,
            0);

        elbowCurrent = new TrapezoidProfile.State(
            ArmConstants.ELBOW_ZERO_REF,
            0);

        shoulderGoal = new TrapezoidProfile.State(
            ArmConstants.SHOULDER_ZERO_REF,
            0);

        shoulderCurrent = new TrapezoidProfile.State(
            ArmConstants.SHOULDER_ZERO_REF,
            0);

        shoulderFF = new ArmFeedforward(
            ArmConstants.SHOULDER_KS,
            ArmConstants.ARM_KG,
            ArmConstants.SHOULDER_KV,
            ArmConstants.SHOULDER_KA);

        elbowFF = new ArmFeedforward(
            ArmConstants.ELBOW_KS,
            ArmConstants.ARM_KG,
            ArmConstants.ELBOW_KV,
            ArmConstants.ELBOW_KA);

        double shoulderMoi = SingleJointedArmSim.estimateMOI( // ethan - good to have, but sometimes
                                                              // we just want to see if the system
                                                              // works code wise in sim so if 
                                                              // you're missing some values you
                                                              // can lie about this one.
            ArmConstants.SHOULDER_ARM_LENGTH,
            ArmConstants.SHOULDER_ARM_MASS);

        shoulderJoint = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ArmConstants.SHOULDER_GEAR_RATIO,
            shoulderMoi,
            ArmConstants.SHOULDER_ARM_LENGTH,
            ArmConstants.SHOULDER_MIN_ANGLE,
            ArmConstants.SHOULDER_MAX_ANGLE,
            true,
            ArmConstants.SHOULDER_ZERO_REF);

        double elbowMoi = SingleJointedArmSim.estimateMOI(
            ArmConstants.ELBOW_ARM_LENGTH,
            ArmConstants.ELBOW_ARM_MASS);

        elbowJoint = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(1),
            ArmConstants.ELBOW_GEAR_RATIO,
            elbowMoi,
            ArmConstants.ELBOW_ARM_LENGTH,
            ArmConstants.ELBOW_MIN_ANGLE,
            ArmConstants.ELBOW_MAX_ANGLE,
            true,
            ArmConstants.ELBOW_ZERO_REF);

        shoulderPID = new PIDController(
            ArmConstants.SHOULDER_KP,
            ArmConstants.SHOULDER_KI,
            ArmConstants.SHOULDER_KD);

        elbowPID = new PIDController(
            ArmConstants.ELBOW_KP,
            ArmConstants.ELBOW_KI,
            ArmConstants.ELBOW_KD);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
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
    public void setShoulderTargetAngle(double angle_rad) { // ethan - looks like you're just setting
                                                           // the current and goal positions, 
                                                           // but aren't actually moving them to
                                                           // the target position? this is 
                                                           // different from the real implementation.
        shoulderGoal = new TrapezoidProfile.State(angle_rad, 0);
        shoulderCurrent = new TrapezoidProfile.State( shoulderJoint.getAngleRads(), shoulderJoint.getVelocityRadPerSec());
    }

    @Override
    public void setElbowTargetAngle(double angle_rad) { // ethan - same as shoulder
        elbowGoal = new TrapezoidProfile.State(angle_rad, 0);
        elbowCurrent = new TrapezoidProfile.State( elbowJoint.getAngleRads(), elbowJoint.getVelocityRadPerSec());
    }

    @Override
    public void swivelElbow() { // ethan - so this is where you move your elbow given the positions
                                // from setElbowTargetAngle()? this method is unnecessary, you
                                // can handle this in setElbowTargetAngle() so real and sim have
                                // the same functionality.
        elbowCurrent = elbowProfile.calculate(
            Constants.globalDelta_s, // ethan - this is how long it takes before robot runs periodic()
                                     // again... you'd create a timer at the moment of setting
                                     // your new target angle and this timer's current time would
                                     // be your argument here. check intake from frc2k26 when you can.
            elbowCurrent,
            elbowGoal);

        double pidOut = elbowPID.calculate(
            elbowJoint.getAngleRads(),
            elbowCurrent.position);

        double ff = elbowFF.calculate(
            elbowCurrent.position,
            elbowCurrent.velocity);

        double voltage = MathUtil.clamp(
            pidOut + ff,
            ArmConstants.LOW_CLAMP, // ethan - volts typically always go from -12 to 12 btw.
            ArmConstants.HIGH_CLAMP);

        setElbowVoltage(voltage);
    }

    @Override
    public void swivelShoulder() { // ethan - same comments from elbow.
        shoulderCurrent = shoulderProfile.calculate(
            Constants.globalDelta_s,
            shoulderCurrent,
            shoulderGoal);

        double pidOut = shoulderPID.calculate(
            shoulderJoint.getAngleRads(),
            shoulderCurrent.position);

        double ff = shoulderFF.calculate(
            shoulderCurrent.position,
            shoulderCurrent.velocity);

        double voltage = MathUtil.clamp(
            pidOut + ff,
            ArmConstants.LOW_CLAMP,
            ArmConstants.HIGH_CLAMP);

        setShoulderVoltage(voltage);
    }

    @Override
    public void setShoulderVoltage(double volts) {
        shoulderMotorVoltage = volts;
        shoulderJoint.setInputVoltage(shoulderMotorVoltage);
    }

    @Override
    public void setElbowVoltage(double volts) {
        elbowMotorVoltage = volts;
        elbowJoint.setInputVoltage(elbowMotorVoltage);
    }

    @Override
    public void setShoulderVelocity(double velocity_rps) {} // ethan - woah this shouldn't be 
                                                            // empty. DCMotorSim has setAngularVelocity()

    @Override
    public void setElbowVelocity(double velocity_rps) {} // ethan - same as shoulder.

    @Override
    public void stopMotor() {
        setShoulderVoltage(0);
        setElbowVoltage(0);
    }
}