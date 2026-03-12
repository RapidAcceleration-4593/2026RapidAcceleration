package frc.robot.commands.auton.center;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class CenterNoPickupNoClimb extends AutonCommand {

    public CenterNoPickupNoClimb(AutonUtil util) {
        super(util, List.of("CenterNoPickup-1"));

        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(10.0));
    }
}
