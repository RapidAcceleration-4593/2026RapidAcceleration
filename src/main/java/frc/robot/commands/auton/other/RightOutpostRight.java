package frc.robot.commands.auton.other;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.*;
import java.util.List;

public class RightOutpostRight extends AutonCommand {
    public RightOutpostRight(AutonUtil util) {
        super(util, List.of("RightOutpost-1", "RightOutpost-2", "RightOutpostRight"));
        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(10.0),
                AutoBuilder.followPath(paths.get(2)),
                NamedCommands.getCommand("ClimbCommand"));
    }
}
