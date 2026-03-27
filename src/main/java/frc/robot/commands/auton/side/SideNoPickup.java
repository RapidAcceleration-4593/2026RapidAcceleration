package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideNoPickup extends AutonCommand {

    public SideNoPickup(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideNoPickup-1"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(7.5));
    }
}
