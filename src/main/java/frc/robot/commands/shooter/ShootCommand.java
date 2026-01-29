package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodConstants;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final IndexerSubsystem indexer;

    public ShootCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.hood = hood;
        this.indexer = indexer;
        addRequirements(shooter, hood, indexer);
    }

    @Override
    public void initialize() {
        shooter.setVelocityCommand(ShooterConstants.kMaximumVelocity);
    }

    @Override
    public void execute() {
        hood.setAngleToHubCommand();
        if (shooter.atTargetVelocity() && hood.atTargetAngle()) {
            indexer.runCommand();
        } else {
            indexer.stopCommand();
        }
    }

    @Override
    public void end(boolean interrputed) {
        shooter.setVelocityCommand(ShooterConstants.kZeroVelocity);
        hood.goToAngleCommand(HoodConstants.kMinimumAngle);
        indexer.stopCommand();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
