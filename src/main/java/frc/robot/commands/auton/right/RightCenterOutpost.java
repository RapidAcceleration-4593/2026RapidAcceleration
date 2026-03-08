package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightCenterOutpost extends AutonCommand {

    public RightCenterOutpost(AutonUtil util) {
        super(util, List.of("RightCenter-1", "RightCenter-2", "RightCenterOutpost-3"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))),
                Commands.waitSeconds(1.0),
                NamedCommands.getCommand("ShootShakeCommand"));
    }
}
