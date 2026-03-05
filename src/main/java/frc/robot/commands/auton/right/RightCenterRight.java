package frc.robot.commands.auton.right;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightCenterRight extends AutonCommand {
    public RightCenterRight(AutonUtil util) {
        super(util, List.of("RightCenter-1", "RightCenter-2", "RightClimbRight"));

        addCommands(
                Commands.race(
                        Commands.waitSeconds(10).andThen(NamedCommands.getCommand("IntakeCommand")),
                        AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommands").withTimeout(Seconds.of(10.0)),
                AutoBuilder.followPath(paths.get(2)),
                NamedCommands.getCommand("ClimbCommand").withTimeout(Seconds.of(5.0))
                // ClimbCommand does nothing now, timeout soon will be useless (hopefully)
                );
    }
}
