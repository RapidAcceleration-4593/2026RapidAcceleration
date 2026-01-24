package frc.robot.commands.deploy;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class SetDeployDistanceCommand extends Command {

    private final DeploySubsystem deploy;
    private final Distance distance;

    public SetDeployDistanceCommand(DeploySubsystem deploy, Distance distance) {
        this.deploy = deploy;
        this.distance = distance;
        addRequirements(deploy);
    }

    @Override
    public void initialize() {
        deploy.setDistance(distance);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
