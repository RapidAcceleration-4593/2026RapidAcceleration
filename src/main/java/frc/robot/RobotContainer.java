package frc.robot;

import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    // public final AprilTagSubsystem apriltag;
    // public final ObjectDetectionSubsystem objectDetection;

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
        // apriltag = AprilTagFactory.initialize(swerve);
        // objectDetection = ObjectDetectionFactory.initialize();

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

        // <------- Experimental ------->
        driverController.a().onTrue(shooter.runCommand());
        driverController.b().whileTrue(indexer.runCommand());
        driverController.y().whileTrue(intake.runCommand());

        // <------- Driver Controller ------->
        driverController.start().onTrue(swerve.resetGyroCommand());

        // driverController.leftBumper().whileTrue(new PathfindCommands().pathfindToOppositeZone(swerve));
        // driverController.leftTrigger(0.5).whileTrue(new DriveToClusterCommand(swerve, objectDetection));

        // <------- Operator Controller ------->
        // operatorController.rightTrigger(0.5).whileTrue(new ShootCommand(shooter, hood, indexer));
        // operatorController
        //         .rightTrigger()
        //         .whileTrue(new SwerveCommands()
        //                 .joystickDrivePointToHub(swerve, driverController::getLeftY, driverController::getLeftX));

        // operatorController.leftTrigger().onTrue(new IntakeCommand(intake, deploy).withName("IntakeCommand"));
        // operatorController.leftBumper().onTrue(new RetractIntakeCommand(intake, deploy).withName("RetractCommand"));
        // operatorController.rightBumper().whileTrue(new ClimbCommand(climber, deploy).withName("ClimbCommand"));
    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        // NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooter, hood, indexer));
        // NamedCommands.registerCommand("IntakeCommand", new RetractIntakeCommand(intake, deploy));
        // NamedCommands.registerCommand("ClimbCommand", new ClimbCommand(climber, deploy));
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        return autonomousChooser.get();
    }
}
