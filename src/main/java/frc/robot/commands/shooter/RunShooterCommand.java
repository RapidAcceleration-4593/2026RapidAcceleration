package frc.robot.commands.shooter;

import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.Simulation;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final IndexerSubsystem indexer;

    private final Simulation simulation;

    public RunShooterCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.hood = hood;
        this.indexer = indexer;

        this.simulation = Simulation.getInstance();
        addRequirements(shooter, hood, indexer);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(kDefaultShooterRPM);
        hood.setAngleFromDistance();
    }

    @Override
    public void execute() {
        if (shooter.atVelocity()) {
            indexer.setSpindexerSpeed(kSpindexerSpeed);
            indexer.setFeederSpeed(kFeederSpeed);

            simulation.launchProjectile(shooter.getVelocity(), hood.getAngle());
        } else {
            indexer.stopSpindexer();
            indexer.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setAngle(kMinimumAngle);
        indexer.stopSpindexer();
        indexer.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
