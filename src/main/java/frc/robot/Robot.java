package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.BuildConstants.*;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.SimulationManager;
import frc.robot.util.mechanism.AngularMechanism3D;
import frc.robot.util.mechanism.LinearMechanism3D;
import frc.robot.util.mechanism.Robot3D;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {

    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private AngularMechanism3D spindexer, shooterBase, hood;
    private LinearMechanism3D intake, climber;

    public Robot() {
        Logger.recordMetadata("ProjectName", MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BUILD_DATE);
        Logger.recordMetadata("GitSHA", GIT_SHA);
        Logger.recordMetadata("GitDate", GIT_DATE);
        Logger.recordMetadata("GitBranch", GIT_BRANCH);
        switch (DIRTY) {
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

        switch (kCurrentMode) {
            case REAL:
                // Running on a real robot, logging to a USB Drive.
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case SIM:
                // Running a physics simulator, logging to NetworkTables.
                Logger.addDataReceiver(new NT4Publisher());
                break;

            case REPLAY:
                // Replacing from a log file.
                setUseTiming(false);
                String logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        // Initialize AdvantageKit Logger.
        Logger.start();
        robotContainer = new RobotContainer();

		SmartDashboard.putNumber("IntakeInches", 0);
		SmartDashboard.putNumber("ClimberInches", 0);
		SmartDashboard.putNumber("ShooterBaseDeg", 0);
		SmartDashboard.putNumber("HoodDeg", 0);
		SmartDashboard.putNumber("SpindexerDeg", 0);

        spindexer = Robot3D.getInstance().getAngularMechanism("Spindexer");
        shooterBase = Robot3D.getInstance().getAngularMechanism("ShooterBase");
        hood = Robot3D.getInstance().getAngularMechanism("Hood");

        intake = Robot3D.getInstance().getLinearMechanism("Intake");
        climber = Robot3D.getInstance().getLinearMechanism("Climber");
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    /** This function is called periodically during all modes. */
    @Override
    public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing.
        Threads.setCurrentThreadPriority(true, 99);

        // Runs the Command Scheduler for polling buttons and managing commands.
        CommandScheduler.getInstance().run();

        // Return to normal thread priority.
        Threads.setCurrentThreadPriority(false, 10);

        spindexer.setAngle(Degrees.of(SmartDashboard.getNumber("SpindexerDeg", 0)));
        shooterBase.setAngle(Degrees.of(SmartDashboard.getNumber("ShooterBaseDeg", 0)));
        hood.setAngle(Degrees.of(SmartDashboard.getNumber("HoodDeg", 0)));
        intake.setDistance(Inches.of(SmartDashboard.getNumber("IntakeInches", 0)));
        climber.setDistance(Inches.of(SmartDashboard.getNumber("ClimberInches", 0)));
    }

    /** This function is called once when the robot is disabled. */
    @Override
    public void disabledInit() {
        if (kCurrentMode == Mode.SIM) {
            SimulationManager.getInstance().resetField();
        }
    }

    /** This function is called periodically when the robot is disabled. */
    @Override
    public void disabledPeriodic() {}

    /** This function runs the autonomous command selected in the RobotContainer class. */
    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        // Schedule the autonomous command.
        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
        // Ensures the autonomous is canceled when teleop begins.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        } else {
            CommandScheduler.getInstance().cancelAll();
        }
    }

    /** This function is called periodically during teleop control. */
    @Override
    public void teleopPeriodic() {}

    /** This function is called once when the test mode is enabled. */
    @Override
    public void testInit() {
        // Cancels all running commands.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    /** This function is called once when the robot is in simulation. */
    @Override
    public void simulationInit() {}

    /** This function is called periodically when in simulaiton. */
    @Override
    public void simulationPeriodic() {
        SimulationManager.getInstance().periodic();
    }
}
