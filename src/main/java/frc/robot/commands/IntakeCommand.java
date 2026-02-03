package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeployConstants;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(deploy.goToDistanceCommand(DeployConstants.kMaximumDistance), intake.runCommand());
    }
}
