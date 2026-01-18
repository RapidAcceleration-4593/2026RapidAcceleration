package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;

    public RunShooterCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {
        this.shooter = shooter;
        this.spindexer = spindexer;
        addRequirements(shooter, spindexer);
    }

    @Override
    public void initialize() {
        shooter.setVelocity(kDefaultShooterRPM.in(RPM));
    }

    @Override
    public void execute() {
        if (shooter.atVelocity()) {
            spindexer.setSpeed(kSpindexerSpeed);
        } else {
            spindexer.stop();
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
        spindexer.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
