package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.turret.TurretSubsystem;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {

    /** Points the hood at the hub, spins up the shooter, and runs the indexer until this Command is canceled. */
    public ShootCommand(
            ShooterSubsystem shooter, TurretSubsystem turret, HoodSubsystem hood, IndexerSubsystem indexer) {
        BooleanSupplier readySupplier =
                () -> shooter.atTargetVelocity() && turret.atTargetAngle() && hood.atTargetAngle();
        addCommands(
                shooter.runCommand(),
                hood.runCommand(),
                indexer.runCommand().onlyWhile(readySupplier).repeatedly());
    }
}
