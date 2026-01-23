package frc.robot.commands.hood;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.hood.HoodSubsystem;

public class ControlHoodCommand extends Command {

    private final HoodSubsystem hood;

    public ControlHoodCommand(HoodSubsystem hood) {
        this.hood = hood;
        addRequirements(hood);
    }

    public void execute() {
        hood.updateControl();
    }

    public boolean isFinished() {
        return false;
    }
}
