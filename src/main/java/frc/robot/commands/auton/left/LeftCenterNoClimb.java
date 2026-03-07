package frc.robot.commands.auton.left;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class LeftCenterNoClimb extends AutonCommand {

    public LeftCenterNoClimb(AutonUtil util) {
        super(util, List.of("LeftCenter-1", "LeftCenter-2"));

        addCommands(
                Commands.race(
                        Commands.waitSeconds(1.0).andThen(NamedCommands.getCommand("IntakeCommand")),
                        AutoBuilder.followPath(paths.get(0))),
                AutoBuilder.followPath(paths.get(1)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(10.0));
    }
}
