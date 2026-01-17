package frc.robot.commands.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;

    public RunShooterCommand(ShooterSubsystem subsystem) {
        this.shooter = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shooter.setTargetVelocity(kDefaultShooterRPM.in(RPM));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
