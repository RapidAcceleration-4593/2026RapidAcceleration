package frc.robot.commands.auton.right;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import frc.robot.util.FieldUtil;
import java.util.List;

public class RightCenterNoClimb extends AutonCommand {

    public RightCenterNoClimb(AutonUtil util) {
        super(util, List.of("RightCenter-1", "RightCenter-2"));

        addCommands(
                Commands.race(AutoBuilder.followPath(paths.get(0)), NamedCommands.getCommand("IntakeCommand")),
                Commands.parallel(
                        AutoBuilder.followPath(paths.get(1)),
                        Commands.either(
                                NamedCommands.getCommand("ShootCommand").withTimeout(10.0),
                                Commands.none(),
                                () -> FieldUtil.isInAllianceZone(AutoBuilder.getCurrentPose()))));
    }
}
