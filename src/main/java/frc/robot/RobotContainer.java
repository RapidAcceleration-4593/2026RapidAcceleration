package frc.robot;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShakeDeployCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.swerve.PathfindCommands;
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

    // Autonomous NetworkTable Instance
    private final NetworkTableInstance networkTableInstance;

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

        networkTableInstance = NetworkTableInstance.getDefault();

        registerCommands();
        configureBindings();
    }

    private void configureBindings() {
        SmartDashboard.putNumber("ExitVelocityFactor", 0.35);
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));
        turret.setDefaultCommand(turret.runToAngleCommand(calculator::getTurretAngle));

        // <------- Driver Controller ------->
        driverController.start().onTrue(swerve.resetGyroCommand());

        driverController
                .rightTrigger(0.5)
                .whileTrue(new ShootCommand(shooter, hood, indexer, calculator)
                        .alongWith(new ShakeDeployCommand(intake, deploy)));
        driverController.leftTrigger(0.5).whileTrue(new IntakeCommand(intake, deploy));

        driverController.leftBumper().whileTrue(new PathfindCommands().pathfindUnderNearestTrench(swerve));
        driverController.rightBumper().onTrue(new RetractIntakeCommand(intake, deploy));

        // <------- Operator Controller ------->
        operatorController.rightTrigger(0.5).whileTrue(shooter.setVoltageCommand(Volts.of(6.0)));
        operatorController.leftTrigger(0.5).whileTrue(intake.runCommand());

        operatorController.leftBumper().whileTrue(turret.setVoltageCommand(Volts.of(-4.0)));
        operatorController.rightBumper().whileTrue(turret.setVoltageCommand(Volts.of(4.0)));

        operatorController.x().whileTrue(deploy.setVoltageCommand(Volts.of(5.0)));
        operatorController.b().whileTrue(deploy.setVoltageCommand(Volts.of(-5.0)));

        operatorController.y().whileTrue(indexer.runCommand());

        // operatorController.povUp().whileTrue(climber.setVoltageCommand(Volts.of(12.0)));
        // operatorController.povDown().whileTrue(climber.setVoltageCommand(Volts.of(-12.0)));

    }

    /** Register NamedCommands to be used in PathPlanner for autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand(
                "ShootCommand",
                new ShootCommand(shooter, hood, indexer, calculator).alongWith(new ShakeDeployCommand(intake, deploy)));
        NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake, deploy).withTimeout(4.5));
        NamedCommands.registerCommand("RetractIntakeCommand", new RetractIntakeCommand(intake, deploy));
        NamedCommands.registerCommand("ClimbCommand", new ClimbCommand(climber));
    }

    /** Select the command to run in autonomous mode. */
    public Command getAutonomousCommand() {
        NetworkTableEntry entry =
                networkTableInstance.getTable("AccelerationStation").getEntry("SelectedAuto");
        String auto = entry.getString("LeftCenterLeft");
        return AutoBuilder.buildAuto(auto);
    }
}
