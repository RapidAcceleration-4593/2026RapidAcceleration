package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.shooter.ShootCommand;
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
import frc.robot.util.mechanism.AngularMechanism3D;
import frc.robot.util.mechanism.Axis;
import frc.robot.util.mechanism.LinearMechanism3D;
import frc.robot.util.mechanism.Robot3D;
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

        driverController = new CommandXboxController(kDriverControllerPort);
        operatorController = new CommandXboxController(kOperatorControllerPort);

        autonomousChooser = new LoggedDashboardChooser<>("Autonomous Routine", AutoBuilder.buildAutoChooser());

        registerCommands();
        configureBindings();
        setupRobot3D();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(new SwerveCommands()
                .joystickDrive(
                        swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));

        driverController
                .rightBumper()
                .whileTrue(new SwerveCommands()
                        .joystickDrivePointToHub(swerve, driverController::getLeftY, driverController::getLeftX));

        driverController.start().onTrue(Commands.runOnce(swerve::resetGyro, swerve));
        driverController.leftTrigger(0.5).whileTrue(new DriveToClusterCommand(swerve, objectDetection));
        driverController.rightTrigger().whileTrue(deploy.goToDistanceCommand(Inches.of(10)));

        operatorController.rightTrigger(0.5).whileTrue(new ShootCommand(shooter, hood, indexer));
    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand("ExampleCommand", Commands.none());
    }

    private void setupRobot3D() {
        Robot3D.getInstance().configure((builder) -> {
            LinearMechanism3D intake = new LinearMechanism3D("Intake", Pose3d.kZero, Axis.X);
            AngularMechanism3D spindexer =
                    new AngularMechanism3D("Spindexer", new Pose3d(0.034, 0, 0, Rotation3d.kZero), Axis.Z);
            AngularMechanism3D shooterBase =
                    new AngularMechanism3D("ShooterBase", new Pose3d(-0.144, 0, 0, Rotation3d.kZero), Axis.Z);
            AngularMechanism3D hood = new AngularMechanism3D(
                    "Hood", new Pose3d(0.1, 0, 0.47, new Rotation3d(0, 0.25, 0)), Axis.Y, shooterBase);
            LinearMechanism3D climber = new LinearMechanism3D("Climber", Pose3d.kZero, Axis.Z);

            builder.addMechanism(intake);
            builder.addMechanism(spindexer);
            builder.addMechanism(shooterBase);
            builder.addMechanism(hood);
            builder.addMechanism(climber);
        });
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        return autonomousChooser.get();
    }
}
