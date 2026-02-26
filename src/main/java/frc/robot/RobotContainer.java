package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShakeDeployCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.shooting.ShotCalculator;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final AprilTagSubsystem apriltag;

    public final ShooterSubsystem shooter;
    public final TurretSubsystem turret;
    public final HoodSubsystem hood;
    public final IndexerSubsystem indexer;

    public final IntakeSubsystem intake;
    public final DeploySubsystem deploy;

    public final ClimberSubsystem climber;

    public final ShotCalculator calculator;

    // Controller(s)
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    // Autonomous Chooser
    private final LoggedDashboardChooser<Command> autonomousChooser;

    public RobotContainer() {
        swerve = SwerveFactory.initialize();
        apriltag = AprilTagFactory.initialize(swerve);

        shooter = ShooterFactory.initialize();
        hood = HoodFactory.initialize();
        turret = TurretFactory.initialize();
        indexer = IndexerFactory.initialize();

        intake = IntakeFactory.initialize();
        deploy = DeployFactory.initialize();

        climber = ClimberFactory.initialize();

        calculator = new ShotCalculator(swerve::getPose, swerve::getChassisSpeeds);

        driverController = new CommandXboxController(kDriverControllerPort);
        operatorController = new CommandXboxController(kOperatorControllerPort);

        autonomousChooser = new LoggedDashboardChooser<>("Autonomous Routine", AutoBuilder.buildAutoChooser());

        registerCommands();
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));
        turret.setDefaultCommand(turret.runToAngleCommand(calculator.calculate().turretAngle()));

        // <------- Experimental ------->
        driverController
                .rightTrigger(0.5)
                .whileTrue(new ShootCommand(shooter, hood, indexer, calculator)
                        .alongWith(new ShakeDeployCommand(intake, deploy)));

        driverController.leftTrigger(0.5).whileTrue(new IntakeCommand(intake, deploy));

        driverController.povLeft().whileTrue(turret.setVoltageCommand(Volts.of(-4)));
        driverController.povRight().whileTrue(turret.setVoltageCommand(Volts.of(4)));

        driverController.povUp().whileTrue(climber.setVoltageCommand(Volts.of(12)));
        driverController.povDown().whileTrue(climber.setVoltageCommand(Volts.of(-12)));

        // <------- Driver Controller ------->
        driverController.start().onTrue(swerve.resetGyroCommand());

        // driverController.leftBumper().whileTrue(new PathfindCommands().pathfindToOppositeZone(swerve));
        // driverController.leftTrigger(0.5).whileTrue(new DriveToClusterCommand(swerve, objectDetection));

        // <------- Operator Controller ------->
        operatorController.rightTrigger(0.5).whileTrue(new ShootCommand(shooter, hood, indexer, calculator));

        operatorController.leftTrigger().onTrue(new IntakeCommand(intake, deploy));
        operatorController.leftBumper().onTrue(new RetractIntakeCommand(intake, deploy));
        operatorController
                .rightBumper()
                .whileTrue(new ClimbCommand(climber).alongWith(new RetractIntakeCommand(intake, deploy)));
    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand("ShootCommand", new ShootCommand(shooter, hood, indexer, calculator));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake, deploy));
        NamedCommands.registerCommand("RetractIntakeCommand", new RetractIntakeCommand(intake, deploy));
        NamedCommands.registerCommand("ClimbCommand", new ClimbCommand(climber));
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        return autonomousChooser.get();
    }
}
