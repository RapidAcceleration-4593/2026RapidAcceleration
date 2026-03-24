package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.deploy.DeployConstants.kMinimumDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractDeployCommand extends SequentialCommandGroup {

    public RetractDeployCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(Commands.parallel(
                deploy.goToDistanceCommand(kMinimumDistance),
                intake.runCommand().onlyWhile(() -> deploy.getCurrentDistance().gt(Inches.of(6.0)))));
    }
}
