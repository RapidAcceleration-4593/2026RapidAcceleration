package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightCenterLoop extends AutonCommand {

    public RightCenterLoop(AutonUtil util) {
        super(util, List.of("RightCenterLoop-1"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                Commands.parallel(
                        NamedCommands.getCommand("ShootCommand"),
                        Commands.waitSeconds(5.0).andThen(NamedCommands.getCommand("ShakeDeployCommand"))));
    }
}
