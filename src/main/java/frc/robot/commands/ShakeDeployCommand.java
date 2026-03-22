package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kAgitationInDistance;
import static frc.robot.subsystems.deploy.DeployConstants.kAgitationOutDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShakeDeployCommand extends ParallelCommandGroup {

    public ShakeDeployCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                Commands.repeatingSequence(
                        deploy.goToDistanceCommand(kAgitationOutDistance, false),
                        deploy.goToDistanceCommand(kAgitationInDistance, false)),
                intake.runCommand());
    }
}
