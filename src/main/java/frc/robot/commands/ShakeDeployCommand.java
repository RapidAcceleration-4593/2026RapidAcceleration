package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.subsystems.deploy.DeployConstants.kAgitationDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.deploy.DeploySubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ShakeDeployCommand extends ParallelCommandGroup {

    private final DeploySubsystem deploy;
    private final IntakeSubsystem intake;

    public ShakeDeployCommand(DeploySubsystem deploy, IntakeSubsystem intake) {
        this.deploy = deploy;
        this.intake = intake;

        addCommands(
                Commands.repeatingSequence(
                        deploy.goToDistanceCommand(kAgitationDistance.plus(Inches.of(2.0))),
                        deploy.goToDistanceCommand(kAgitationDistance.minus(Inches.of(2.0)))),
                intake.runCommand());
    }
}
