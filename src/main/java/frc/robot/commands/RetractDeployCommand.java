package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.*;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractDeployCommand extends SequentialCommandGroup {

    public RetractDeployCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                Commands.race(
                        deploy.goToDistanceCommand(kMinimumDistance, true),
                        intake.runCommand()
                                .onlyWhile(() -> deploy.getCurrentDistance().gt(kMinimumIntakeDistance))),
                new ShakeDeployCommand(intake, deploy));
    }
}
