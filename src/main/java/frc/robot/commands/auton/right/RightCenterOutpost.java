package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightCenterOutpost extends AutonCommand {

    public RightCenterOutpost(AutonUtil util) {
        super(util, List.of("RightCenter-1", "CenterOutpost-1", "CenterOutpost-2"));

        addCommands(
                Commands.race(AutoBuilder.followPath(paths.get(0)), NamedCommands.getCommand("IntakeCommand")),
                AutoBuilder.followPath(paths.get(1)),
                Commands.race(NamedCommands.getCommand("IntakeCommand"), Commands.waitSeconds(1.25)),
                Commands.parallel(AutoBuilder.followPath(paths.get(2)), NamedCommands.getCommand("ShootShakeCommand")));
    }
}
