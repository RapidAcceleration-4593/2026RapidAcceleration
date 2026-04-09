package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kMaximumDistance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class ExtendDeployCommand extends SequentialCommandGroup {

    public ExtendDeployCommand(DeploySubsystem deploy) {
        addCommands(deploy.goToDistanceCommand(kMaximumDistance, true));
    }
}
