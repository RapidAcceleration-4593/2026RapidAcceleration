package frc.robot.commands.auton.left;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class Left2xCenterNoClimb extends AutonCommand {

    public Left2xCenterNoClimb(AutonUtil util) {
        super(util, List.of("LeftCenter-1", "LeftCenter-2", "LeftCenter-3", "LeftCenter-4"));

        addCommands(
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(5.0),
                NamedCommands.getCommand("IntakeCommand").withDeadline(AutoBuilder.followPath(paths.get(2))),
                AutoBuilder.followPath(paths.get(3)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(7.5));
    }
}
