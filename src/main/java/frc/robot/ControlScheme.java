package frc.robot;

import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.PathingMode;
import frc.robot.subsystems.drive.PathingOverride;
import frc.robot.subsystems.drive.SwerveInput;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.hand.Hand;
import frc.robot.superstructure.IntentionStates;
import frc.robot.superstructure.SS;
import frc.robot.util.IPeriodic;
import frc.robot.util.Util;

public class ControlScheme implements IPeriodic {

    protected Drive drive;
    protected Arm arm;
    protected Elevator elevator;
    protected Hand hand;
    protected Climber climber;
    protected SS ss;
    
    private boolean intook;
    private boolean defend = false;
    private boolean prevDPadLeft = false;

    private double angle = 45;

    public boolean firstIntakeUp;
    public ControlScheme() {
        super();
        drive = Drive.getInstance();
        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        hand = Hand.getInstance();
        climber = Climber.getInstance();
        ss = SS.getInstance();
        intook = false;
        firstIntakeUp = true;
    }

    public void init() {
        drive.queueState(PathingMode.FIELD_RELATIVE);
        drive.setPathingOverride(PathingOverride.NONE);
        System.out.println("Control Scheme Initialized");
    }
    

    public void periodic() {
        double rotMult = 1.0;
        double mult = 0;
        if (OI.DR.getLeftTriggerAxis() >= 0.8) {
            mult = -0.5;
        } else {
            mult = -1.0;
        }

        // if (OI.DR.getRightTriggerAxis() >= 0.8) {
        // // drive.queueState(PathingMode.TRACKING);
        // drive.setPathingOverride(PathingOverride.SHOOTING);

        // } else {
        // drive.queueState(PathingMode.FIELD_RELATIVE);
        // drive.setPathingOverride(PathingOverride.NONE);

        // }
        double x_ = OI.deadband(OI.DR.getLeftY() * mult);
        double y_ = OI.deadband(OI.DR.getLeftX() * mult);
        double w_ = rotMult * -Util.sqInput(OI.deadband(OI.DR.getRightX()));
        double throttle = Util.sqInput(1.0
                - OI.deadband(Math.max(OI.DR.getLeftTriggerAxis(), OI.DR.getRightTriggerAxis())));

        SwerveInput input = new SwerveInput(x_, y_, w_, throttle);
        drive.setInput(input);

        if (OI.DR.getAButtonPressed()) {
            defend = !defend;
        }

        // if (OI.DR.getAButton()) {
        //     servo.trackUp();
        //     // servo.zero(45);
        // } else if (OI.DR.getBButton()) {
        //     servo.trackDown();
        //     // servo.zero(45);
        // } else if (OI.DR.getYButton()) {
        //     servo.zero(45);
        // }



        // D-pad up (POV == 0) → zero intake encoder to down angle

    }
}