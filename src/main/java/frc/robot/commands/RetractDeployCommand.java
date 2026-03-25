package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.deploy.DeployConstants.kMinimumDistance;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class RetractDeployCommand extends ParallelCommandGroup {

    public RetractDeployCommand(IntakeSubsystem intake, DeploySubsystem deploy) {
        addCommands(
                deploy.goToDistanceCommand(kMinimumDistance, true),
                intake.runCommand().onlyWhile(() -> deploy.getCurrentDistance().gt(Inches.of(6.0))));
    }
}
