package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightNoPickup extends AutonCommand {

    public RightNoPickup(AutonUtil util) {
        super(util, List.of("RightNoPickup-1"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(7.5));
    }
}
