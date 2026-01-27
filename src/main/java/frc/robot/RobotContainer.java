package frc.robot;

import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve.DriveToClusterCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionSubsystem;
import frc.robot.util.mechanism.AngularMechanism3D;
import frc.robot.util.mechanism.Axis;
import frc.robot.util.mechanism.LinearMechanism3D;
import frc.robot.util.mechanism.Robot3D;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final AprilTagSubsystem apriltag;
    public final ObjectDetectionSubsystem objectDetection;

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(kOperatorControllerPort);

    public RobotContainer() {
        swerve = SwerveFactory.initialize();
        apriltag = AprilTagFactory.initialize(swerve);
        objectDetection = ObjectDetectionFactory.initialize();

        registerCommands();
        configureBindings();
        setupRobot3D();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));

        driverController.start().onTrue(Commands.runOnce(swerve::resetGyro, swerve));
        driverController.leftTrigger(0.5).whileTrue(new DriveToClusterCommand(swerve, objectDetection));
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
        return AutoBuilder.buildAuto("Example");
    }
}
