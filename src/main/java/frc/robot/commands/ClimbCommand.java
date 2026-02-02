package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.deploy.DeployConstants;
import frc.robot.subsystems.deploy.DeploySubsystem;

public class ClimbCommand extends SequentialCommandGroup {

    public ClimbCommand(ClimberSubsystem climber, DeploySubsystem deploy) {
        addCommands(
                Commands.parallel(
                        deploy.goToDistanceCommand(DeployConstants.kRetractedDistance),
                        climber.goToDistanceCommand(ClimberConstants.kMaximumDistance)),
                climber.goToDistanceCommand(ClimberConstants.kMinimumDistance));
    }
}
