package frc.robot.commands.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;

import java.util.Set;

public class PathfindCommand extends DeferredCommand {

    public PathfindCommand(
            SwerveSubsystem swerve,
            Pose2d targetPose) {
        super(
                () -> {
                    PathConstraints constraints =
                            new PathConstraints(kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);
                    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
                },
                Set.of(swerve));
    }
}
