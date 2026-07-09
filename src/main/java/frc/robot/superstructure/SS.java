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
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberStates;
import frc.robot.subsystems.elevator.Elevator;
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

    private static Arm arm;
    private static Elevator elevator;
    private static Hand hand;
    private static Climber climber;
    private static Vision vision;
    private static Drive drive;

    private Alert unimplementedStateAlert = new Alert("Unimplemented internal State", AlertType.kError);

    private SS() {
        super("SS");
        intention = IntentionStates.IDLE;
        queueState(InternalStates.IDLE);
        booted = false;
        homed = false;

        arm = Arm.getInstance();
        elevator = Elevator.getInstance();
        hand = Hand.getInstance();
        climber = Climber.getInstance();
        vision = Vision.getInstance();
        drive = Drive.getInstance();
    }

    private boolean isHomed() {
        if (climber.isHomed() && hand.isHomed()) {
            return true;
        }

        return false;
    }

    public InternalStates defaultIntentionHandling() {
        return switch (intention) {
            case IDLE -> InternalStates.IDLE;
            case CLIMB -> InternalStates.CLIMB1;
            case GRIPPING_CORAL -> InternalStates.GRIPPING_CORAL1;
            case GRIPPING_ALGAE -> InternalStates.GRIPPING_ALGAE1;
            case RELEASING -> InternalStates.RELEASING;
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
                break;
            case GRIPPING_ALGAE1:
                break;
            default:
                unimplementedStateAlert.set(true);
                break;
        }
    }

    @Override
    public void inputPeriodic() {
    }

    @Override
    protected void outputPeriodic() {
    }

}