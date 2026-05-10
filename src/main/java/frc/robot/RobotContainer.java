package frc.robot;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.Controllers.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;
import frc.robot.commands.leds.*;
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
import frc.robot.subsystems.vision.quest.QuestNavSubsystem;
import frc.robot.util.FieldUtil;

public class RobotContainer {

    // Subsystem(s)
    public final SwerveSubsystem swerve;
    public final QuestNavSubsystem questNav;

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

    public RobotContainer() {
        swerve = SwerveFactory.initialize();
        questNav = QuestNavFactory.initialize(swerve);
        swerve.setVisionResetCallback(questNav::resetPose);

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

        FieldUtil.setPoseSupplier(swerve::getPose);

        configureBindings();
    }

    private void configureBindings() {
        swerve.setDefaultCommand(SwerveCommands.joystickDrive(
                swerve,
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                () -> FieldUtil.isInAllianceZone()
                        && shooter.getTargetVelocity().gt(RPM.zero())));
        turret.setDefaultCommand(turret.runToAngleCommand(calculator::getTurretAngle));

        // <------- Driver Controller ------->
        driverController.start().onTrue(swerve.resetGyroCommand());
        driverController.a().onTrue(swerve.resetPoseCommand());
        driverController.x().onTrue(swerve.stopXCommand());
        driverController.y().whileTrue(new RetractDeployCommand(deploy).alongWith(new RunIntakeLEDLayer(LEDs)));
        driverController.povDown().whileTrue(indexer.runReverseCommand());

        driverController
                .rightTrigger(0.5)
                .whileTrue(new ShootCommand(shooter, turret, hood, indexer, calculator)
                        .alongWith(new ShakeDeployCommand(intake, deploy))
                        .alongWith(new RunShooterLEDLayer(LEDs))
                        .alongWith(new RunWarningLEDLayer(LEDs)
                                .onlyWhile(() -> !turret.atTargetAngle())
                                .repeatedly()));
        driverController
                .rightBumper()
                .whileTrue(new ShootCommand(shooter, turret, hood, indexer, calculator)
                        .alongWith(new IntakeCommand(intake, deploy))
                        .alongWith(new RunShooterLEDLayer(LEDs))
                        .alongWith(new RunWarningLEDLayer(LEDs)
                                .onlyWhile(() -> !turret.atTargetAngle())
                                .repeatedly()));

        driverController
                .leftTrigger(0.5)
                .whileTrue(new IntakeCommand(intake, deploy).alongWith(new RunIntakeLEDLayer(LEDs)));

        // <------- Operator Controller ------->
        operatorController.rightTrigger(0.5).whileTrue(new ManualShootCommand(shooter, hood, indexer));
        operatorController.leftTrigger(0.5).whileTrue(intake.runCommand());

        operatorController.leftBumper().whileTrue(turret.setVoltageCommand(Volts.of(-4.0)));
        operatorController.rightBumper().whileTrue(turret.setVoltageCommand(Volts.of(4.0)));

        operatorController
                .a()
                .whileTrue(new OuttakeCommand(intake, deploy)
                        .alongWith(new OuttakeShootCommand(shooter, turret, hood, indexer, calculator)
                                .onlyWhile(FieldUtil::isInNeutralZone))
                        .alongWith(new RunIntakeLEDLayer(LEDs)));
        operatorController.x().whileTrue(deploy.setVoltageCommand(Volts.of(5.0)));
        operatorController.b().whileTrue(deploy.setVoltageCommand(Volts.of(-5.0)));

        operatorController.povUp().whileTrue(hood.setVoltageCommand(Volts.of(4.0)));
        operatorController.povDown().whileTrue(hood.setVoltageCommand(Volts.of(-4.0)));

        operatorController.start().onTrue(turret.runOnce(() -> turret.setDefaultCommand(turret.idle())));
        operatorController
                .back()
                .onTrue(turret.runOnce(
                        () -> turret.setDefaultCommand(turret.runToAngleCommand(calculator::getTurretAngle))));
    }

    /** Select the command to run in Autonomous. */
    public Command getAutonomousCommand() {
        return Commands.none();
    }
}
