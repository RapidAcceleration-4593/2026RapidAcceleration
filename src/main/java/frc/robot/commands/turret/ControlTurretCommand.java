package frc.robot.commands.turret;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.TurretSubsystem;

public class ControlTurretCommand extends Command {

    private final TurretSubsystem turret;

    public ControlTurretCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double desired = turret.calculateDesiredAngle();
        double wrapped = turret.wrapToSafeRange(turret.getAngle().in(Degrees), desired);

        turret.setAngle(Degrees.of(wrapped));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
