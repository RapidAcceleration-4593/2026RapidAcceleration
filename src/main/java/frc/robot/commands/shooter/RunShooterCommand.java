package frc.robot.commands.shooter;

import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.util.SimulationManager;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final IndexerSubsystem indexer;
    private final SimulationManager simulation;

    private double lastShotTime;

    public RunShooterCommand(ShooterSubsystem shooter, HoodSubsystem hood, IndexerSubsystem indexer) {
        this.shooter = shooter;
        this.hood = hood;
        this.indexer = indexer;
        this.simulation = SimulationManager.getInstance();
        addRequirements(shooter, hood, indexer);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(kDefaultShooterRPM);
        lastShotTime = -1.0;
    }

    @Override
    public void execute() {
        hood.setAngleToHub();

        double now = Timer.getFPGATimestamp();
        boolean canShoot = lastShotTime < 0 || now - lastShotTime >= 0.25;

        if (shooter.atVelocity() && hood.atAngle()) {
            indexer.setSpindexerSpeed(kSpindexerSpeed);
            indexer.setFeederSpeed(kFeederSpeed);

            if (canShoot) {
                simulation.launchProjectile(hood.getAngle());
                lastShotTime = now;
            }
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
