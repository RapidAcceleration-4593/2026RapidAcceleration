package frc.robot;

import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.swerve.DriveToClusterCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final AprilTagSubsystem apriltag;
    public final ObjectDetectionSubsystem objectDetection;

    public final ShooterSubsystem shooter;
    public final HoodSubsystem hood;
    public final IndexerSubsystem indexer;

    public final IntakeSubsystem intake;
    public final DeploySubsystem deploy;

    // public final ClimberSubsystem climber;

    // Controller(s)
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    // Autonomous Chooser
    private final LoggedDashboardChooser<Command> autonomousChooser;

    public RobotContainer() {
        swerve = SwerveFactory.initialize();
        apriltag = AprilTagFactory.initialize(swerve);
        objectDetection = ObjectDetectionFactory.initialize();

        shooter = ShooterFactory.initialize();
        hood = HoodFactory.initialize(swerve);
        indexer = IndexerFactory.initialize();

        intake = IntakeFactory.initialize();
        deploy = DeployFactory.initialize();

        // climber = ClimberFactory.initialize();

        driverController = new CommandXboxController(kDriverControllerPort);
        operatorController = new CommandXboxController(kOperatorControllerPort);

        autonomousChooser = new LoggedDashboardChooser<>("Autonomous Routine", AutoBuilder.buildAutoChooser());

        registerCommands();
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(new SwerveCommands()
                .joystickDrive(
                        swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));

        driverController
                .rightBumper()
                .whileTrue(new SwerveCommands()
                        .joystickDrivePointToHub(swerve, driverController::getLeftY, driverController::getLeftX));

        driverController.start().onTrue(swerve.resetGyroCommand());
        driverController.leftTrigger(0.5).whileTrue(new DriveToClusterCommand(swerve, objectDetection));

        operatorController.rightTrigger(0.5).whileTrue(new ShootCommand(shooter, hood, indexer));
        operatorController.leftTrigger(0.5).whileTrue(new IntakeCommand(intake, deploy));
        // operatorController.rightBumper().whileTrue(new ClimbCommand(climber, deploy));
    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooter, hood, indexer).withTimeout(4.0));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake, deploy).withTimeout(3.0));
        // NamedCommands.registerCommand("ClimbCommand", new ClimbCommand(climber, deploy));
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        return autonomousChooser.get();
    }
}
