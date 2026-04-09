package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kMinimumDistance;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class RetractDeployCommand extends SequentialCommandGroup {

    public RetractDeployCommand(DeploySubsystem deploy) {
        addCommands(deploy.goToDistanceCommand(kMinimumDistance, false));
    }
}
