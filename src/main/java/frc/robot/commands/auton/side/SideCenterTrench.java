package frc.robot.commands.auton.side;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class SideCenterTrench extends AutonCommand {

    public SideCenterTrench(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("SideCenterTrench-1", "SideCenterTrench-2"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand"));
    }
}
