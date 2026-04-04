package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideCenterBump extends AutonCommand {

    public SideCenterBump(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterBump-1", "SideCenterBump-2", "SideCenterBump-3", "SideCenterBump-4"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))),
                NamedCommands.getCommand("ShootCommand").withTimeout(8.0),
                AutoBuilder.followPath(paths.get(2)),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(3))));
    }
}
