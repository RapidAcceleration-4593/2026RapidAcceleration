package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShakeDeployCommand extends ParallelCommandGroup {

    public ShakeDeployCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                Commands.repeatingSequence(
                        deploy.goToDistanceCommand(kMaximumDistance, false).withTimeout(0.5),
                        deploy.goToDistanceCommand(kAgitationDistance, false).withTimeout(0.5)),
                intake.runCommand());
    }
}
