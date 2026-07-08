// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandStates;

public class SS extends StateMachineSubsystemBase<SuperstructureStates>{
    private static SS instance;

    public static final double STOWED_ELEVATOR_POS = 0.0;
    public static final double STOWED_SHOULDER_DEG = 0.0;
    public static final double STOWED_ELBOW_DEG = 0.0;

    public static final double INTAKE_ELEVATOR_POS = 5.0;
    public static final double INTAKE_SHOULDER_DEG = 30.0;
    public static final double INTAKE_ELBOW_DEG = 45.0;

    public static final double SCORE_ELEVATOR_POS = 20.0;
    public static final double SCORE_SHOULDER_DEG = 60.0;
    public static final double SCORE_ELBOW_DEG = 90.0;

    private enum Scoring {
        RAISING,
        SETTLING,
        READY
    }

    private static final double SETTLE_TIME_S = 0.2;

    private Scoring scoringSubstate;
    private boolean scoringSubstateFirstLoop;
    private final Timer substateTimer = new Timer();

    private final EnumSet<Flag> flags = EnumSet.noneOf(Flag.class);

    public enum Flag {
        HOME,
        SCORE_LOW,
        SCORE_HIGH,
        MANUAL_UP,
        MANUAL_DOWN,
        CLIMB
    }

    private final Elevator elevator;
    private final Arm arm;
    private final Hand hand;
    private final Climber climber;

    private double scoreElevatorTarget = STOWED_ELEVATOR_POS;

    private SS() {
        super("SS");
        this.elevator = Elevator.getInstance();
        this.arm = Arm.getInstance();
        this.hand = Hand.getInstance();
        this.climber = Climber.getInstance();
        queueState(SuperstructureStates.STOWED);
    }

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }


    public void enable(Flag flag) {
        flags.add(flag);
    }

    public void disable(Flag flag) {
        flags.remove(flag);
    }

    public void set(Flag flag, boolean active) {
        if (active) {
            flags.add(flag);
        } else {
            flags.remove(flag);
        }
    }

    public void toggle(Flag flag) {
        set(flag, !has(flag));
    }

    public boolean has(Flag flag) {
        return flags.contains(flag);
    }

    private void setScoringSubstate(Scoring next) {
        if (scoringSubstate != next) {
            scoringSubstate = next;
            scoringSubstateFirstLoop = true;
            substateTimer.restart();
        } else {
            scoringSubstateFirstLoop = false;
        }
    }

    private boolean substateElapsed(double seconds) {
        return substateTimer.hasElapsed(seconds);
    }

    @Override
    public void handleStateMachine() {
        
        if (has(Flag.CLIMB)) {
            queueState(SuperstructureStates.CLIMBING);
        } else if (has(Flag.HOME)) {
            queueState(SuperstructureStates.STOWED);
        } else if (has(Flag.MANUAL_UP) || has(Flag.MANUAL_DOWN)) {
            
            queueState(SuperstructureStates.STOWED);

            
        } else if (has(Flag.SCORE_HIGH) || has(Flag.SCORE_LOW)) {
            queueState(SuperstructureStates.SCORE);
        } else {
            queueState(SuperstructureStates.STOWED);
        }

        switch (getState()) {
            case STOWED:
                elevator.setTargetPosition(STOWED_ELEVATOR_POS);
                arm.setShoulderTargetAngle(INTAKE_SHOULDER_DEG);
                arm.setElbowTargetAngle(STOWED_ELBOW_DEG);
                break;

            case INTAKE_CORAL:
                elevator.setTargetPosition(INTAKE_ELEVATOR_POS);
                arm.setShoulderTargetAngle(INTAKE_SHOULDER_DEG);
                arm.setElbowTargetAngle(INTAKE_ELBOW_DEG);
                hand.requestState(HandStates.GRIPPING_CORAL);
                break;

            case SCORE:
                handleScoring();
                break;

            case CLIMBING:
                handleClimbing();
                break;

            default:
                break;
        }
    }

    private void handleClimbing() {
        elevator.setTargetPosition(STOWED_ELEVATOR_POS);
        arm.setShoulderTargetAngle(STOWED_SHOULDER_DEG);
        arm.setElbowTargetAngle(STOWED_ELBOW_DEG);

        boolean clearToClimb = elevator.isState(ElevatorStates.IDLE)
                && arm.isState(ArmStates.IDLE);

        if (clearToClimb) {
            climber.requestState(ClimberStates.SHALLOW_CLIMB_TRAVELLING);
        } else {
            climber.requestState(ClimberStates.IDLE);
        }
    }

    private void handleScoring() {
        if (stateInit()) {
            setScoringSubstate(Scoring.RAISING);
        }

        scoreElevatorTarget = SCORE_ELEVATOR_POS;

        switch (scoringSubstate) {
            case RAISING:
                driveToScorePosition();
                if (atScorePosition()) {
                    setScoringSubstate(Scoring.SETTLING);
                }
                break;

            case SETTLING:
                driveToScorePosition();
                if (!atScorePosition()) {
                    setScoringSubstate(Scoring.RAISING);
                } else if (substateElapsed(SETTLE_TIME_S)) {
                    setScoringSubstate(Scoring.READY);
                }
                break;

            case READY:
                driveToScorePosition();
                if (!atScorePosition()) {
                    setScoringSubstate(Scoring.RAISING);
                }
                break;
        }
    }

    //67

    private void driveToScorePosition() {
        elevator.setTargetPosition(scoreElevatorTarget);
        arm.setShoulderTargetAngle(SCORE_SHOULDER_DEG);
        arm.setElbowTargetAngle(SCORE_ELBOW_DEG);
    }

    private boolean atScorePosition() {
        return elevator.isState(ElevatorStates.IDLE)
                && arm.isState(ArmStates.IDLE);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Superstructure/State", getState());
        String[] activeFlags = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", activeFlags);
    }
    
}