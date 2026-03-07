package frc.robot.commands.auton.right;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.auton.AutonCommand;
import frc.robot.commands.auton.AutonUtil;
import java.util.List;

public class RightNoPickupRight extends AutonCommand {
    public RightNoPickupRight(AutonUtil util) {
        super(util, List.of("RightNoPickup-1", "RightNoPickupRight-2"));
        addCommands(
                AutoBuilder.followPath(paths.get(0)),
                NamedCommands.getCommand("ShootShakeCommand").withTimeout(Seconds.of(10.0)),
                AutoBuilder.followPath(paths.get(1)),
                // climber arm is raised at a specific event marker, using a new named command (and actually a rather
                // simple one)
                NamedCommands.getCommand(
                        "ClimbCommand") // I can not imagine the climbCommand having no timeout automatically
                );
    }
}
