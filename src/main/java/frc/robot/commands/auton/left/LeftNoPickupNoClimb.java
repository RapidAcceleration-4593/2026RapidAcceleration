package frc.robot.commands.auton.left;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class LeftNoPickupNoClimb extends AutonCommand {

    public LeftNoPickupNoClimb(AutonUtil util) {
        super(util, List.of("LeftNoPickup-1"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(5.0));
    }
}
