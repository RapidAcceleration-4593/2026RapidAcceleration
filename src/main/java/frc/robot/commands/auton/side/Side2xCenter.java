package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Side2xCenter extends AutonCommand {

    public Side2xCenter(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenter-1", "SideCenter-2", "SideCenter-3", "SideCenter-4"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(4.0),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(2))),
                AutoBuilder.followPath(paths.get(3)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(7.5));
    }
}
