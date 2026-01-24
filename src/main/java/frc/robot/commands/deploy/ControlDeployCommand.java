package frc.robot.commands.deploy;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class ControlDeployCommand extends Command {

    private final DeploySubsystem deploy;

    public ControlDeployCommand(DeploySubsystem deploy) {
        this.deploy = deploy;
        addRequirements(deploy);
    }

    @Override
    public void execute() {
        deploy.updateControl();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
