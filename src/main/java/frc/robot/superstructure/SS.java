// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
// Raymond: IState is never used in here, delete it.
import frc.robot.util.IState;
import frc.robot.util.StateMachineSubsystemBase;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorStates;
import frc.robot.subsystems.Elbow.Elbow;
import frc.robot.subsystems.Elbow.ElbowStates;
import frc.robot.subsystems.Shoulder.Shoulder;
// Raymond: ShoulderState is singular but everything else is plural - ElbowStates, HandStates, ClimberStates, ElevatorStates. rename it ShoulderStates so it matches.
import frc.robot.subsystems.Shoulder.ShoulderState;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandStates;

// Raymond: run spotlessApply - missing space before the brace, and there's stray double blank lines and trailing whitespace all through this file.
public class SS extends StateMachineSubsystemBase<SuperstructureStates>{
    private static SS instance;

    // Raymond: these setpoints belong in each subsystem's own Constants file (see how the reference pulls ShooterConstants/ClimberConstants), not dumped in SS. and _POS isn't a unit - name it _m or whatever the elevator actually reads in.
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

    // Raymond: this whole Flag-bag isn't how we do it. look at the reference - it splits Intention (what the driver asks for) from InternalState (what the machine's actually doing), and intend(Intention) + handleIntention() maps one to the other. here you've got a pile of booleans you cascade through, so two flags can be set at once and whoever's first in the if-chain wins. switch to the intention pattern.
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
    private final Shoulder shoulder;
    private final Elbow elbow;
    private final Hand hand;
    private final Climber climber;

    private double scoreElevatorTarget = STOWED_ELEVATOR_POS;

    private SS() {
        super("SS");
        this.elevator = Elevator.getInstance();
        this.shoulder = Shoulder.getInstance();
        this.elbow = Elbow.getInstance();
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
        
        // Raymond: nothing in this cascade ever queues INTAKE_CORAL, so that case in the switch below is dead - you can't reach it. wire a flag/intention to it or drop it.
        if (has(Flag.CLIMB)) {
            queueState(SuperstructureStates.CLIMBING);
        } else if (has(Flag.HOME)) {
            queueState(SuperstructureStates.STOWED);
        // Raymond: this branch just queues STOWED and does nothing - manual up/down are wired to nothing. either implement them or delete the flags. and kill the random blank lines inside.
        } else if (has(Flag.MANUAL_UP) || has(Flag.MANUAL_DOWN)) {
            
            queueState(SuperstructureStates.STOWED);

            
        // Raymond: SCORE_HIGH and SCORE_LOW both go to SCORE, and scoreElevatorTarget is always SCORE_ELEVATOR_POS - so high vs low is identical, the distinction does nothing. the reference parameterizes the setpoint off the intention. do that instead of two flags that behave the same.
        } else if (has(Flag.SCORE_HIGH) || has(Flag.SCORE_LOW)) {
            queueState(SuperstructureStates.SCORE);
        } else {
            queueState(SuperstructureStates.STOWED);
        }

        switch (getState()) {
            case STOWED:
                elevator.setTargetPosition(STOWED_ELEVATOR_POS);
                shoulder.setTargetAngle(STOWED_SHOULDER_DEG);
                elbow.setTargetAngle(STOWED_ELBOW_DEG);
                break;

            case INTAKE_CORAL:
                elevator.setTargetPosition(INTAKE_ELEVATOR_POS);
                shoulder.setTargetAngle(INTAKE_SHOULDER_DEG);
                elbow.setTargetAngle(INTAKE_ELBOW_DEG);
                // Raymond: you drive hand with requestState() but elevator/shoulder/elbow with setTargetPosition/setTargetAngle - pick one way to talk to subsystems. the reference just calls queueState() on the subsystem directly, the requestState wrapper is pointless indirection.
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
        shoulder.setTargetAngle(STOWED_SHOULDER_DEG);
        elbow.setTargetAngle(STOWED_ELBOW_DEG);

        // Raymond: using isState(IDLE) to mean "got there" is fragile - a subsystem sits in IDLE for other reasons too. the reference checks isValueReached(tolerance). use that here and in atScorePosition().
        boolean clearToClimb = elevator.isState(ElevatorStates.IDLE)
                && shoulder.isState(ShoulderState.IDLE)
                && elbow.isState(ElbowStates.IDLE);

        if (clearToClimb) {
            climber.requestState(ClimberStates.SHALLOW_CLIMBING);
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

            // Raymond: READY just holds position - nothing ever tells the hand to release/score the piece. so SCORE never actually scores anything. finish it.
            case READY:
                driveToScorePosition();
                if (!atScorePosition()) {
                    setScoringSubstate(Scoring.RAISING);
                }
                break;
        }
    }

    // Raymond: what's //67? leftover junk, delete it.

    private void driveToScorePosition() {
        elevator.setTargetPosition(scoreElevatorTarget);
        shoulder.setTargetAngle(SCORE_SHOULDER_DEG);
        elbow.setTargetAngle(SCORE_ELBOW_DEG);
    }

    private boolean atScorePosition() {
        return elevator.isState(ElevatorStates.IDLE)
                && shoulder.isState(ShoulderState.IDLE)
                && elbow.isState(ElbowStates.IDLE);
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("Superstructure/State", getState());
        String[] activeFlags = flags.stream().map(Enum::name).toArray(String[]::new);
        Logger.recordOutput("Superstructure/Flags", activeFlags);
    }
    
}