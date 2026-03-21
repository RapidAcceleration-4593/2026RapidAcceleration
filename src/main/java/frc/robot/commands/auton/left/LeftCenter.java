package frc.robot.commands.auton.left;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class LeftCenter extends AutonCommand {

    public LeftCenter(AutonUtil util) {
        super(util, List.of("LeftCenter-1", "LeftCenter-2"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(10.0));
    }
}
