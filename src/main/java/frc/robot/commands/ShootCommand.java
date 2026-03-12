package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.leds.RunWarningLEDPatternCommand;
import frc.robot.subsystems.ShotCalculatorSubsystem;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.leds.LEDSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(
            ShooterSubsystem shooter,
            HoodSubsystem hood,
            IndexerSubsystem indexer,
            ShotCalculatorSubsystem calculator,
            TurretSubsystem turret,
            LEDSubsystem LEDs) {

        addCommands(
                shooter.runAtVelocityCommand(calculator::getShooterVelocity),
                hood.runToAngleCommand(calculator::getHoodAngle),
                new RunWarningLEDPatternCommand(LEDs).onlyWhile(() -> !turret.atTargetAngle() || !calculator.isValid()),
                Commands.repeatingSequence(Commands.sequence(Commands.waitSeconds(0.5), indexer.runCommand())
                        .onlyWhile(calculator::isValid)));
    }
}
