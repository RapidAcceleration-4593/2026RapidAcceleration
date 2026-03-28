package frc.robot.commands.auton.center;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class CenterNoPickupNoTraversal extends AutonCommand {

    public CenterNoPickupNoTraversal(AutonUtil util) {
        super(util, false, List.of("CenterNoPickup-1"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(7.5));
    }
}
