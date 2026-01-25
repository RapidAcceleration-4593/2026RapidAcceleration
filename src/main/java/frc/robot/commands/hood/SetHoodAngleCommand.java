package frc.robot.commands.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class SetHoodAngleCommand extends Command {

    private final HoodSubsystem hood;
    private final Angle angle;

    public SetHoodAngleCommand(HoodSubsystem hood, Angle angle) {
        this.hood = hood;
        this.angle = angle;
        addRequirements(hood);
    }

    public void initialize() {
        hood.setAngle(angle);
    }

    public boolean isFinished() {
        return true;
    }
}
