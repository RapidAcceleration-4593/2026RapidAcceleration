package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightCenterOutpost extends AutonCommand {

    public RightCenterOutpost(AutonUtil util) {
        super(util, List.of("RightCenter-1", "CenterOutpost-1", "CenterOutpost-2"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                AutoBuilder.followPath(paths.get(1)),
                AutoBuilder.followPath(paths.get(2)));
    }
}
