package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberConstants;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbCommand extends SequentialCommandGroup {

    public ClimbCommand(ClimberSubsystem climber) {
        addCommands(
                climber.goToDistanceCommand(ClimberConstants.kMaximumDistance),
                Commands.waitSeconds(3.0),
                climber.goToDistanceCommand(ClimberConstants.kMinimumDistance));
    }
}
