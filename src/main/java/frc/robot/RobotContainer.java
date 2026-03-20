package frc.robot;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Controllers.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RetractIntakeCommand;
import frc.robot.commands.ShakeDeployCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.auton.AutonManager;
import frc.robot.commands.leds.RunIntakeLEDPatternCommand;
import frc.robot.commands.leds.RunShooterLEDPatternCommand;
import frc.robot.commands.swerve.PathfindCommands;
import frc.robot.commands.swerve.SwerveCommands;
import frc.robot.factory.*;
import frc.robot.subsystems.ShotCalculatorSubsystem;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.FieldUtil;

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
    public final LEDSubsystem LEDs;

    public final ShotCalculatorSubsystem calculator;

    // Controller(s)
    private final CommandXboxController driverController;
    private final CommandXboxController operatorController;

    // Autonomous Selector
    private final AutonManager autonManager;
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
        LEDs = new LEDSubsystem();

        calculator = new ShotCalculatorSubsystem(swerve::getPose, swerve::getChassisSpeeds);

        driverController = new CommandXboxController(kDriverControllerPort);
        operatorController = new CommandXboxController(kOperatorControllerPort);

        autonManager = new AutonManager(swerve);
        autonManager.warmup();
        networkTableInstance = NetworkTableInstance.getDefault();

        registerCommands();
        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve, driverController::getLeftY, driverController::getLeftX, driverController::getRightX));
        turret.setDefaultCommand(turret.runToAngleCommand(calculator::getTurretAngle));

        // <------- Driver Controller ------->
        driverController.start().onTrue(swerve.resetGyroCommand());
        driverController.x().onTrue(swerve.stopXCommand());

        driverController
                .rightTrigger(0.5)
                .whileTrue(new ShootCommand(shooter, hood, indexer, calculator)
                        .alongWith(new ShakeDeployCommand(intake, deploy))
                        .alongWith(new RunShooterLEDPatternCommand(LEDs)));
        driverController
                .rightBumper()
                .whileTrue(new IntakeCommand(intake, deploy)
                        .alongWith(new ShootCommand(shooter, hood, indexer, calculator))
                        .alongWith(new RunShooterLEDPatternCommand(LEDs)));

        driverController
                .leftTrigger(0.5)
                .whileTrue(new IntakeCommand(intake, deploy).alongWith(new RunIntakeLEDPatternCommand(LEDs)));
        driverController.leftBumper().whileTrue(new PathfindCommands().pathfindUnderNearestTrench(swerve));

        driverController.y().onTrue(new RetractIntakeCommand(intake, deploy));

        // <------- Operator Controller ------->
        operatorController.rightTrigger(0.5).whileTrue(shooter.setVoltageCommand(Volts.of(6.0)));
        operatorController.leftTrigger(0.5).whileTrue(intake.runCommand());

        operatorController.leftBumper().whileTrue(turret.setVoltageCommand(Volts.of(-4.0)));
        operatorController.rightBumper().whileTrue(turret.setVoltageCommand(Volts.of(4.0)));

        operatorController.x().whileTrue(deploy.setVoltageCommand(Volts.of(5.0)));
        operatorController.b().whileTrue(deploy.setVoltageCommand(Volts.of(-5.0)));

        operatorController.y().whileTrue(indexer.runCommand());
        operatorController.a().whileTrue(intake.setVoltageCommand(Volts.of(-6.0)));

        operatorController.start().onTrue(turret.runOnce(() -> turret.setDefaultCommand(turret.idle())));

        operatorController
                .back()
                .onTrue(turret.runOnce(
                        () -> turret.setDefaultCommand(turret.runToAngleCommand(calculator::getTurretAngle))));

        new Trigger(indexer::getShotDetected).onTrue(Commands.runOnce(() -> {
            if (FieldUtil.isInAllianceZone(swerve.getPose())) {
                indexer.addHubShot();
            } else {
                indexer.addFeedingShot();
            }
        }));
    }

    /** Select the command to run in Autonomous. */
    public Command getAutonomousCommand() {
        NetworkTableEntry entry =
                networkTableInstance.getTable("AccelerationStation").getEntry("SelectedAuto");
        String name = entry.getString("DoNothing");
        return autonManager.getAuton(name);
    }

    /** Register NamedCommands for Autonomous. */
    private void registerCommands() {
        NamedCommands.registerCommand(
                "ShootCommand",
                new ShootCommand(shooter, hood, indexer, calculator)
                        .alongWith(new RunShooterLEDPatternCommand(LEDs))); // .until(indexer::isFuelDetected)
        NamedCommands.registerCommand(
                "ShootShakeCommand",
                new ShootCommand(shooter, hood, indexer, calculator)
                        .alongWith(new ShakeDeployCommand(intake, deploy))
                        .alongWith(new RunShooterLEDPatternCommand(LEDs)));
        NamedCommands.registerCommand(
                "IntakeCommand", new IntakeCommand(intake, deploy).alongWith(new RunIntakeLEDPatternCommand(LEDs)));
        NamedCommands.registerCommand("ShakeDeployCommand", new ShakeDeployCommand(intake, deploy));
    }
}
