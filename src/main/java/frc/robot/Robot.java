
// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import frc.robot.subsystems.Elbow.Elbow;
import frc.robot.subsystems.Elbow.ElbowReal;
import frc.robot.subsystems.Elbow.ElbowSim;
import frc.robot.subsystems.Elevator.Elevator;
import frc.robot.subsystems.Elevator.ElevatorReal;
import frc.robot.subsystems.Elevator.ElevatorSimulation;
import frc.robot.subsystems.Shoulder.Shoulder;
import frc.robot.subsystems.Shoulder.ShoulderReal;
import frc.robot.subsystems.Shoulder.ShoulderSim;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.util.MTimer;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this
 * class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private Elevator elevator;
    private Elbow elbow;
    private Shoulder shoulder;

    private boolean lastState = false;

    private MTimer pipelineSwitch = new MTimer();

    //private CommandXboxController cmdController = new CommandXboxController(0);

    /**
     * This function is run when the robot is first started up and should be used
     * for any initialization code.
     */
    @Override
    public void robotInit() {

        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        switch (Constants.currentMode) {
            case REAL:
                // Running on a real robot, log to a USB stick ("/U/logs")
                elevator = new Elevator(new ElevatorReal());
                elbow = new Elbow(new ElbowReal());
                shoulder = new Shoulder(new ShoulderReal());
                Logger.addDataReceiver(new WPILOGWriter("U/logs/" + BuildConstants.GIT_BRANCH));
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, log to NT
                elevator = new Elevator(new ElevatorSimulation());
                elbow = new Elbow(new ElbowSim());
                shoulder = new Shoulder(new ShoulderSim());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replaying a log, set up replay source
                elevator = new Elevator(new ElevatorSimulation());
                elbow = new Elbow(new ElbowSim());
                shoulder = new Shoulder(new ShoulderSim());
                setUseTiming(false); // Run as fast as possible
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;

        }
        
        // See http://bit.ly/3YIzFZ6 for more information on timestamps in AdvantageKit.
        // Logger.disableDeterministicTimestamps()

        // Start AdvantageKit logger
        Logger.start();

        

        // init subsystems
  
        // Check for valid swerve config
        var modules = new SwerveModuleConstants[] {
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight
        };
        for (var constants : modules) {
            if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
                    || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
                throw new RuntimeException(
                        "You are using an unsupported swerve configuration, which this template does not support without manual customization. The 2025 release of Phoenix supports some swerve configurations which were not available during 2025 beta testing, preventing any development and support from the AdvantageKit developers.");
            }
        }

        

    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);
        elevator.periodic();
        elbow.periodic();
        shoulder.periodic();
        PerfTracker.periodic();
        Threads.setCurrentThreadPriority(false, 10);
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        pipelineSwitch.reset();
    }

    /** This function is called periodically when disabled. */
    @Override
    public void disabledPeriodic() {
        boolean buttonPressed = RobotController.getUserButton();

        if (buttonPressed && !lastState) {
        }

        lastState = buttonPressed;

        if(pipelineSwitch.after(0.5)) {
            pipelineSwitch.reset();
            
        }

        OI.DR.setRumble(RumbleType.kBothRumble, 0);
    }

    /** Runs at the start of auto */
    @Override
    public void autonomousInit() {
        // maybe put something
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
        // run auto!
    }

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
       
    }

    /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
        if(RobotBase.isSimulation()){

        } else if(OI.DR.getAButtonReleased()) {
            
        }
    }

    /** This function is called once when test mode is enabled. */
    @Override
    public void testInit() {
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }

    /** This function is called once when the robot is first started up. */
    @Override
    public void simulationInit() {
    }

    /** This function is called periodically whilst in simulation. */
    @Override
    public void simulationPeriodic() {
        // drive.updateSimulationField();
    }
}
