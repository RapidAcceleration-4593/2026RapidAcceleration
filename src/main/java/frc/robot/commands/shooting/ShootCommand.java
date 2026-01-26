package frc.robot.commands.shooting;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import java.util.function.BooleanSupplier;

public class ShootCommand extends ParallelCommandGroup {

    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {
        BooleanSupplier ready = () -> shooter.atTargetVelocity() && hood.atTargetAngle();

        addCommands(
                shooter.setVelocity(ShooterConstants.kMaximumVelocity),
                hood.setAngleToHubCommand(),
                Commands.either(indexer.run(), indexer.stop(), ready).repeatedly());

        this.finallyDo(interrupted -> {
            indexer.stop();
            shooter.setVelocity(ShooterConstants.kMinimumVelocity);
            hood.setTargetAngle(HoodConstants.kMinimumAngle);
        });
    }
}
