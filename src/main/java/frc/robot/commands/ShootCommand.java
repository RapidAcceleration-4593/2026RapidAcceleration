package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

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
            TurretSubsystem turret,
            LEDSubsystem LEDs,
            ShotCalculatorSubsystem calculator) {

        addCommands(
                // shooter.runAtVelocityCommand(calculator::getShooterVelocity),
                shooter.setVoltageCommand(Volts.of(7.5)),
                hood.runToAngleCommand(calculator::getHoodAngle),
                new RunWarningLEDPatternCommand(LEDs)
                        .onlyWhile(() -> !calculator.isValid() || !turret.atTargetAngle())
                        .repeatedly(),
                Commands.waitSeconds(0.5).andThen(indexer.runCommand()));
    }
}
