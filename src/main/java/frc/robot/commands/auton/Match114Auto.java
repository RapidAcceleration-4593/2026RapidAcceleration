package frc.robot.commands.auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;

public class Match114Auto extends AutonCommand {

    public Match114Auto(AutonUtil util, boolean isFlipped) {
        super(util, isFlipped, List.of("Match114-1", "Match114-2"));

        addCommands(
                Commands.deadline(
                        Commands.waitSeconds(5.5),
                        NamedCommands.getCommand("ShootCommand"),
                        Commands.sequence(
                                NamedCommands.getCommand("ExtendDeployCommand"),
                                NamedCommands.getCommand("RetractDeployCommand"))),
                Commands.race(AutoBuilder.followPath(paths.get(0)), NamedCommands.getCommand("IntakeCommand")),
                AutoBuilder.followPath(paths.get(1)),
                Commands.parallel(
                        NamedCommands.getCommand("ShootCommand"), NamedCommands.getCommand("ShakeDeployCommand")));
    }
}
