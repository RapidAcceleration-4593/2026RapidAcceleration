package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kMaximumDistance;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends ParallelCommandGroup {

    public IntakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(deploy.goToDistanceCommand(kMaximumDistance, false), intake.runCommand());
    }
}
