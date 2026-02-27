package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.shooting.ShotCalculator;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {

    /** Points the hood at the hub, spins up the shooter, and runs the indexer until this Command is canceled. */
    public ShootCommand(
            ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer, ShotCalculator calculator) {
        BooleanSupplier readySupplier =
                () -> shooter.atTargetVelocity() && hood.atTargetAngle(); // && turret.atTargetAngle()
        addCommands(
                shooter.runAtVelocityCommand(() -> calculator.calculate().shooterVelocity()),
                hood.runToAngleCommand(() -> calculator.calculate().hoodAngle()),
                indexer.runCommand().onlyWhile(readySupplier).repeatedly());
    }
}
