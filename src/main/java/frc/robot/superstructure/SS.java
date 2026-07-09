// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.superstructure;

import java.util.EnumSet;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.StateMachineSubsystemBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmStates;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorConstants;
import frc.robot.subsystems.elevator.ElevatorStates;
import frc.robot.subsystems.hand.Hand;
import frc.robot.subsystems.hand.HandStates;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.drive.*;
import frc.robot.util.MTimer;
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

    private MTimer timer = new MTimer();

    private boolean booted;
    private boolean homed;

    private double elevatorTargetHeight_m;

    private static Arm arm;
    private static Elevator elevator;
    private static Hand hand;
    private static Climber climber;
    private static Vision vision;
    private static Drive drive;
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
        vision = Vision.getInstance();
        drive = Drive.getInstance();
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
            case CLIMB1:
                queueState(switch (intention) {
                    case CLIMB -> InternalStates.CLIMB1;
                    default -> defaultIntentionHandling();
                });
            case GRIPPING_CORAL1:
                queueState(switch (intention) {
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
                    default -> defaultIntentionHandling();
                });
            case GRIPPING_ALGAE1:
                queueState(switch (intention) {
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
                    default -> defaultIntentionHandling();
                });

            case CLIMB2:
                queueState(switch (intention) {
                    case CLIMB -> InternalStates.CLIMB2;
                    default -> defaultIntentionHandling();
                });
            case GRIPPING_CORAL2:
                queueState(switch (intention) {
                    case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL2;
                    default -> defaultIntentionHandling();
                });
            case GRIPPING_ALGAE2:
                queueState(switch (intention) {
                    case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE2;
                    default -> defaultIntentionHandling();
                });
            
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
                climber.queueState(ClimberStates.SHALLOW_CLIMB_TRAVELLING);
                if (climber.isClimbComplete()) {
                    queueState(InternalStates.CLIMB2);
                }

            case CLIMB2:
                climber.queueState(ClimberStates.RELEASING);
                break;

            case GRIPPING_CORAL1:
                elevator.queueState(ElevatorStates.TRAVELLING);
                arm.queueState(ArmStates.TRAVELLING_TO_POSITION);
                if (elevator.isAtTargetPosition() && arm.isAtTargetPosition()) {
                    queueState(InternalStates.GRIPPING_CORAL2);
                }
                break;
            
            case GRIPPING_CORAL2:
                hand.queueState(HandStates.GRIPPING_CORAL);
                readyToScore = true;
                break;

            case GRIPPING_ALGAE1:
                elevator.queueState(ElevatorStates.TRAVELLING);
                arm.queueState(ArmStates.TRAVELLING_TO_POSITION);
                if (elevator.isAtTargetPosition() && arm.isAtTargetPosition()) {
                    queueState(InternalStates.GRIPPING_ALGAE2);
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
                elevator.queueState(ElevatorStates.TRAVELLING);
                arm.setArmTargetAngle(ArmConstants.ShoulderTargetAngle_Deg, ArmConstants.ElbowTargetAngle_Deg, true);
                arm.queueState(ArmStates.HOLDING_POSITION);

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
                elevator.queueState(ElevatorStates.TRAVELLING);
                arm.setArmTargetAngle(ArmConstants.ShoulderTargetAngle_Deg, ArmConstants.ElbowTargetAngle_Deg, true);
                arm.queueState(ArmStates.HOLDING_POSITION);

                
                if (elevator.isAtTargetPosition()) {
                    queueState(InternalStates.IDLE);
                }

            case RELEASING:
                arm.queueState(ArmStates.HOLDING_POSITION);
                hand.queueState(HandStates.RELEASING);
                readyToScore = true;
                break;


            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }

    public void intend(IntentionStates intention) {
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

    }

}