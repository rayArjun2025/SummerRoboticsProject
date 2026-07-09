package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ClimberIOSim implements ClimberIO {
    private double climberVoltage = 0.0;

    private final PIDController climberPid;
    private final TrapezoidProfile climbProfile;
    private final ArmFeedforward climberFF;
    private final SingleJointedArmSim climberSim;

    private TrapezoidProfile.State climbGoal;
    private TrapezoidProfile.State climbCurrent;

    public ClimberIOSim() {
        climbProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(ClimberConstants.MAX_VELOCITY, ClimberConstants.MAX_ACCELERATION));

        climbGoal = new TrapezoidProfile.State(0, 0);
        climbCurrent = new TrapezoidProfile.State(0, 0);

        climberFF = new ArmFeedforward(
            ClimberConstants.climberKS,
            ClimberConstants.climberKG,
            ClimberConstants.climberKV,
            ClimberConstants.climberKA);

        climberPid = new PIDController(
            ClimberConstants.climberKP,
            ClimberConstants.climberKI,
            ClimberConstants.climberKD);

        double moi = SingleJointedArmSim.estimateMOI(
            ClimberConstants.ARM_LENGTH,
            ClimberConstants.ARM_MASS);

        climberSim = new SingleJointedArmSim(
            DCMotor.getKrakenX60Foc(2),
            ClimberConstants.GEAR_RATIO,
            moi,
            ClimberConstants.ARM_LENGTH,
            Math.toRadians(ClimberConstants.MIN_DEG),
            Math.toRadians(ClimberConstants.MAX_DEG),
            true,
            0);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climberSim.update(Constants.globalDelta_s);

        inputs.connected = true;
        inputs.climberCurrent = climberSim.getCurrentDrawAmps();
        inputs.climberPositionDeg = Math.toDegrees(climberSim.getAngleRads());
        inputs.climberVelocity_dps = Math.toDegrees(climberSim.getVelocityRadPerSec());
        inputs.climberVoltage = climberVoltage;
    }

    @Override
    public void setClimberVoltage(double voltage) {
        climberVoltage = MathUtil.clamp(voltage, ClimberConstants.LOW_CLAMP, ClimberConstants.HIGH_CLAMP);
        climberSim.setInputVoltage(climberVoltage);
    }

    @Override
    public void setClimberVelocity(double velocity_rps) {}

    @Override
    public void stopClimb() {
        setClimberVoltage(0);
    }

    @Override
    public void climbTo() {
        climbCurrent = climbProfile.calculate(Constants.globalDelta_s, climbCurrent, climbGoal);

        double pidOut = climberPid.calculate(climberSim.getAngleRads(), climbCurrent.position);
        double ff = climberFF.calculate(climbCurrent.position, climbCurrent.velocity);
        double voltage = MathUtil.clamp(pidOut + ff, ClimberConstants.LOW_CLAMP, ClimberConstants.HIGH_CLAMP);

        setClimberVoltage(voltage);
    }

    @Override
    public void setTargetAngle(double target_deg) {
        climbGoal = new TrapezoidProfile.State(Math.toRadians(target_deg), 0);
    }
}