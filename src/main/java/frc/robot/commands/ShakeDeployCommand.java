package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.deploy.DeployConstants.kAgitationDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShakeDeployCommand extends ParallelCommandGroup {

    public ShakeDeployCommand(DeploySubsystem deploy, IntakeSubsystem intake) {

        addCommands(Commands.repeatingSequence(
                deploy.goToDistanceCommand(kAgitationDistance.plus(Inches.of(2.0)))
                        .withTimeout(0.5),
                Commands.waitSeconds(0.25),
                deploy.goToDistanceCommand(kAgitationDistance.minus(Inches.of(2.0)))
                        .withTimeout(0.5),
                Commands.waitSeconds(0.25)));
    }
}
