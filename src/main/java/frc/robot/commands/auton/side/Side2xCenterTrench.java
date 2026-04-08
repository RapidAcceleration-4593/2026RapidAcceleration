package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Side2xCenterTrench extends AutonCommand {

    public Side2xCenterTrench(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterTrench-1", "SideCenterTrench-2", "SideCenterTrench-3"));

        addCommands(
                Commands.race(
                        AutoBuilder.followPath(paths.get(0)),
                        NamedCommands.getCommand("ExtendDeployCommand")
                                .andThen(NamedCommands.getCommand("IntakeCommand"))),
                NamedCommands.getCommand("ShootCommand").withTimeout(5.0),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(1))),
                AutoBuilder.followPath(paths.get(2)),
                NamedCommands.getCommand("ShootCommand"));
    }
}
