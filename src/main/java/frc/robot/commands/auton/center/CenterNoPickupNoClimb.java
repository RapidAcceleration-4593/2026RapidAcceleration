package frc.robot.commands.auton.center;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class CenterNoPickupNoClimb extends AutonCommand {

    public CenterNoPickupNoClimb(AutonUtil util) {
        super(util, List.of("NoPickup-1"));

        addCommands(Commands.parallel(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootCommand").withTimeout(5.0)));
    }
}
