package frc.robot.commands.auton.other;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class CenterNoPickupLeft extends AutonCommand {

    public CenterNoPickupLeft(AutonUtil util) {
        super(util, List.of("CenterNoPickup-1", "CenterNoPickupLeft"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(5.0),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ClimbCommand"));
    }
}
