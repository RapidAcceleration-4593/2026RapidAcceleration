package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightOutpostNoClimb extends AutonCommand {

    public RightOutpostNoClimb(AutonUtil util) {
        super(util, List.of("RightOutpost-1", "RightOutpost-2"));

        addCommands(
                Commands.race(AutoBuilder.followPath(paths.get(0)), NamedCommands.getCommand("IntakeCommand")),
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(1)),
                        NamedCommands.getCommand("ShootCommand").withTimeout(10.0)));
    }
}
