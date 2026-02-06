package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ClimbCommand extends SequentialCommandGroup {

    public ClimbCommand(ClimberSubsystem climber) {
        addCommands();
    }
}
