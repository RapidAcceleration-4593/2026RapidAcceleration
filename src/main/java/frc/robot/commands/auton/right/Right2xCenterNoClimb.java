package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Right2xCenterNoClimb extends AutonCommand {

    public Right2xCenterNoClimb(AutonUtil util) {
        super(util, List.of("RightCenter-1", "RightCenter-2", "RightCenter-3", "RightCenter-4"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(3.0),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(2))),
                AutoBuilder.followPath(paths.get(3)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(7.5));
    }
}
