package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Side2xCenterTrench extends AutonCommand {

    public Side2xCenterTrench(AutonUtil util, boolean isFlipped) {
        super(
                util,
                isFlipped,
                List.of("SideCenterTrench-1", "SideCenterTrench-2", "SideCenterTrench-3", "SideCenterTrench-4"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootCommand").withTimeout(5.0),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(2))),
                AutoBuilder.followPath(paths.get(3)),
                NamedCommands.getCommand("ShootCommand"));
    }
}
