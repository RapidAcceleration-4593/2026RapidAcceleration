package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RunShooterCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double speed;

    public RunShooterCommand(ShooterSubsystem subsystem, double speed) {
        this.shooter = subsystem;
        this.speed = speed;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        shooter.setMotorSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
