package frc.robot;

import static frc.robot.Constants.Controllers.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.shooter.RunShooterCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionSubsystem;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final AprilTagSubsystem apriltag;
    public final ObjectDetectionSubsystem objectDetection;
    public final ShooterSubsystem shooter;

    // Controller(s)
    private final CommandXboxController driverController = new CommandXboxController(kDriverControllerPort);
    private final CommandXboxController operatorController = new CommandXboxController(kOperatorControllerPort);

    public RobotContainer() {
        swerve = SwerveFactory.initialize();
        apriltag = AprilTagFactory.initialize(swerve);
        objectDetection = ObjectDetectionFactory.initialize(swerve);
        shooter = ShooterFactory.initialize();

        registerCommands();
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));

        driverController.start().onTrue(Commands.runOnce(swerve::resetGyro, swerve));

        operatorController.rightTrigger(0.5).whileTrue(new RunShooterCommand(shooter, kShooterSpeed));
    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand("ExampleCommand", Commands.none());
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        return AutoBuilder.buildAuto("Example");
    }
}
