package frc.robot.commands;

import static frc.robot.subsystems.deploy.DeployConstants.kMaximumDistance;
import static frc.robot.subsystems.deploy.DeployConstants.kMinimumDistance;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbCommand extends SequentialCommandGroup {

    public ClimbCommand(ClimberSubsystem climber) {
        addCommands(
                climber.goToDistanceCommand(kMaximumDistance),
                Commands.waitSeconds(3.0),
                climber.goToDistanceCommand(kMinimumDistance));
    }
}
