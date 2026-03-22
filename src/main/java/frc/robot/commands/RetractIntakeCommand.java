package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kMinimumDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractIntakeCommand extends SequentialCommandGroup {

    public RetractIntakeCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                Commands.waitSeconds(3.0), deploy.goToDistanceCommand(kMinimumDistance, true), intake.stopCommand());
    }
}
