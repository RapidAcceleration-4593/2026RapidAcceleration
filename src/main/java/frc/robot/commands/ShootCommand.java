package frc.robot.commands;

import static frc.robot.subsystems.shooter.ShooterConstants.kShootVelocity;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends ParallelCommandGroup {

    /** Points the hood at the hub, spins up the shooter, and runs the indexer until this Command is canceled. */
    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {
        addCommands(
                shooter.runAtVelocityCommand(kShootVelocity),
                // hood.pointAtHubCommand(),
                indexer.runCommand().onlyWhile(shooter::atTargetVelocity));
    }
}
