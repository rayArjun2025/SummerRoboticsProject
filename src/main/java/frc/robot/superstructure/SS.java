// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import org.littletonrobotics.junction.Logger;

import frc.robot.util.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandConstants;
import frc.robot.subsystems.hand.HandStates;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class SS extends StateMachineSubsystemBase<InternalStates>{

    private static SS instance;

    public static SS getInstance() {
        if (instance == null) {
            instance = new SS();
        }
        return instance;
    }

    private IntentionStates intention;

    private boolean booted;
    private boolean homed;

    private double elevatorTargetHeight_m;

    private static Arm arm;
    private static Elevator elevator;
    private static Hand hand;
    private static Climber climber;
    private static boolean readyToScore;

    private Alert unimplementedStateAlert = new Alert("Unimplemented internal State", AlertType.kError);

    private SS() {
        super("SS");
        intention = IntentionStates.IDLE;
        queueState(InternalStates.IDLE);
        booted = false;
        homed = false;
        readyToScore = false;
        elevatorTargetHeight_m = 0.0;

        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        hand = Hand.getInstance();
        climber = Climber.getInstance();
    }


    public InternalStates defaultIntentionHandling() {
        return switch (intention) {
            case IDLE -> InternalStates.IDLE;
            case CLIMB -> InternalStates.CLIMB1;
            case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
            case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
            case RELEASING -> InternalStates.RELEASING;
            case RAISING -> InternalStates.RAISING;
            case LOWERING -> InternalStates.LOWERING;
        };
    }

    private void handleIntention() {
        switch (getState()) { //ts internal
            case BOOT:
                break;
            case DISABLED:
                break;
            case IDLE: 
                queueState(switch (intention) {
                    case IDLE -> InternalStates.IDLE; //case Idle = intention, Internalstates.idle = internal
                    default -> defaultIntentionHandling();
                });
                break;

            case CLIMB1:
                queueState(switch (intention) {
                    case CLIMB -> InternalStates.CLIMB1;
                    default -> defaultIntentionHandling();
                });
                break;
            
            case CLIMB2:
                queueState(switch (intention) {
                    case CLIMB -> InternalStates.CLIMB2;
                    default -> defaultIntentionHandling();
                });
                break;

            case GRIPPING_CORAL1:
                queueState(switch (intention) {
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
                    case RELEASING -> InternalStates.RELEASING;
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
                    default -> defaultIntentionHandling();
                });
                break;

            case GRIPPING_CORAL2:
                queueState(switch (intention) {
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL2;
                    case RELEASING -> InternalStates.RELEASING;
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
                    default -> defaultIntentionHandling();
                });
                break;

            case GRIPPING_ALGAE1:
                queueState(switch (intention) {
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
                    case RELEASING -> InternalStates.RELEASING;
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
                    default -> defaultIntentionHandling();
                });
                break;

            case GRIPPING_ALGAE2:
                queueState(switch (intention) {
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE2;
                    case RELEASING -> InternalStates.RELEASING;
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
                    default -> defaultIntentionHandling();
                });
                break;

            case LOWERING:
                queueState(switch (intention) {
                    case LOWERING -> InternalStates.LOWERING;
                    case RAISING -> InternalStates.RAISING;
                    default -> defaultIntentionHandling();
                });
                break;

            case RAISING:
                queueState(switch (intention) {
                    case LOWERING -> InternalStates.LOWERING;
                    case RAISING -> InternalStates.RAISING;
                    default -> defaultIntentionHandling();
                });
                break;
            
            case RELEASING:
                queueState(switch (intention) {
                    case RELEASING -> InternalStates.RELEASING;
                    default -> defaultIntentionHandling();
                });
                break;

            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }



    @Override
    public void handleStateMachine() {
        if (!booted && !isState(InternalStates.DISABLED)) {
            queueState(InternalStates.BOOT);
            hand.queueState(HandStates.HOMING);
            climber.queueState(ClimberStates.HOMING);
        }

        handleIntention();

        switch (getState()) {
            case DISABLED:
                break;
            case BOOT:
                if (isHomed()) {
                    homed = true;
                    booted = true;
                    queueState(InternalStates.IDLE);
                }
                else {
                    homed = false;
                    booted = true;
                    hand.queueState(HandStates.HOMING);
                    climber.queueState(ClimberStates.HOMING);
                }
                break;
            case IDLE:
                arm.queueState(ArmStates.IDLE);
                climber.queueState(ClimberStates.IDLE);
                elevator.queueState(ElevatorStates.IDLE);
                hand.queueState(HandStates.IDLE);
                readyToScore = false;
                break;
                
            case CLIMB1:
                climber.setTargetAngle(ClimberConstants.MAX_DEG);
                if (climber.isClimbComplete()) {
                    queueState(InternalStates.CLIMB2);
                }
                break;

            case CLIMB2:
                climber.queueState(ClimberStates.RELEASING);
                break;

            case GRIPPING_CORAL1:
                if (!arm.isAtTargetPosition() || !elevator.isAtTargetPosition()) {
                    elevator.setTargetPosition(ElevatorConstants.CoralHeight_m);
                    arm.setArmTargetAngle(ArmConstants.CoralShoulderAngle_Deg, ArmConstants.CoralElbowAngle_Deg,true);

                }
                else {
                    queueState(InternalStates.GRIPPING_CORAL2);
                }
                break;
            
            case GRIPPING_CORAL2:
                hand.queueState(HandStates.GRIPPING_CORAL);
                readyToScore = true;
                break;

            case GRIPPING_ALGAE1:
                if (!arm.isAtTargetPosition() || !elevator.isAtTargetPosition()) {
                    elevator.setTargetPosition(ElevatorConstants.AlgaeHeight_m);
                    arm.setArmTargetAngle(ArmConstants.AlgaeShoulderAngle_Deg, ArmConstants.AlgaeElbowAngle_Deg, true);
                }
                else {
                    queueState(InternalStates.GRIPPING_CORAL2);
                }
                break;
            
            case GRIPPING_ALGAE2:
                hand.queueState(HandStates.GRIPPING_ALGAE);
                readyToScore = true;
                break;
            
            case RAISING:
                if (elevatorTargetHeight_m == 0)
                    elevatorTargetHeight_m = ElevatorConstants.LevelOneTargetHeight_m;
                else if (elevatorTargetHeight_m == ElevatorConstants.LevelOneTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelTwoTargetHeight_m;
                else if (elevatorTargetHeight_m == ElevatorConstants.LevelTwoTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelThreeTargetHeight_m;
                else if (elevatorTargetHeight_m == ElevatorConstants.LevelThreeTargetHeight_m || elevatorTargetHeight_m == ElevatorConstants.LevelFourTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelFourTargetHeight_m;
                
                elevator.setTargetPosition(elevatorTargetHeight_m);
                arm.setArmTargetAngle(ArmConstants.ShoulderTargetAngle_Deg, ArmConstants.ElbowTargetAngle_Deg, true);

                if (elevator.isAtTargetPosition()) {
                    queueState(InternalStates.IDLE);
                }

                break;

            case LOWERING:
                if (elevatorTargetHeight_m == ElevatorConstants.LevelFourTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelThreeTargetHeight_m;
                else if (elevatorTargetHeight_m == ElevatorConstants.LevelThreeTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelTwoTargetHeight_m;
                else if (elevatorTargetHeight_m == ElevatorConstants.LevelTwoTargetHeight_m || elevatorTargetHeight_m == ElevatorConstants.LevelOneTargetHeight_m)
                    elevatorTargetHeight_m = ElevatorConstants.LevelOneTargetHeight_m;
                else if (elevatorTargetHeight_m == 0.0)
                    elevatorTargetHeight_m = 0.0;

                elevator.setTargetPosition(elevatorTargetHeight_m);
                arm.setArmTargetAngle(ArmConstants.ShoulderTargetAngle_Deg, ArmConstants.ElbowTargetAngle_Deg, true);

                
                if (elevator.isAtTargetPosition()) {
                    queueState(InternalStates.IDLE);
                }
                break;

            case RELEASING:
                arm.queueState(ArmStates.HOLDING_POSITION);
                hand.queueState(HandStates.RELEASING);
                readyToScore = true;

                if (hand.isAtTargetPosition(HandConstants.home_deg)) {
                    queueState(InternalStates.IDLE);
                }
                break;


            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }

    public void intend(IntentionStates intention) {
        System.out.println("New intention: " + intention);
        this.intention = intention;
    }

    public IntentionStates getIntention() {
        return intention;
    }


    private boolean isHomed() {
        if (climber.isHomed() && hand.isHomed()) {
            return true;
        }

        return false;
    }

    @Override
    public void inputPeriodic() {
    }

    @Override
    protected void outputPeriodic() {
        Logger.recordOutput("SS/Booted?", booted);
        Logger.recordOutput("SS/Intention", intention);
        Logger.recordOutput("SS/Homed?", isHomed());
        Logger.recordOutput("SS/ArmState", arm.getState());
        Logger.recordOutput("SS/ElevatorState", elevator.getState());
        Logger.recordOutput("SS/ClimberState", climber.getState());
    }

}