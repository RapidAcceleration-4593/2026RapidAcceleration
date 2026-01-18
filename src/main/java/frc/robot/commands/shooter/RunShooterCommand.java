package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;
    private final SpindexerSubsystem spindexer;

    public RunShooterCommand(ShooterSubsystem shooter, HoodSubsystem hood, SpindexerSubsystem spindexer) {
        this.shooter = shooter;
        this.hood = hood;
        this.spindexer = spindexer;
        addRequirements(shooter, hood, spindexer);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(kDefaultShooterRPM);
        hood.setAngleFromDistance();
    }

    @Override
    public void execute() {
        if (shooter.atVelocity()) {
            spindexer.setSpindexerSpeed(kSpindexerSpeed);
            spindexer.setFeederSpeed(kFeederSpeed);
        } else {
            spindexer.stopSpindexer();
            spindexer.stopFeeder();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        hood.setAngle(Degrees.zero());
        spindexer.stopSpindexer();
        spindexer.stopFeeder();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
