package frc.robot.commands.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.Set;

public class PathfindCommand extends DeferredCommand {

    public PathfindCommand(
            SwerveSubsystem swerve,
            Pose2d targetPose,
            double velocity,
            double acceleration,
            double angularVelocity,
            double angularAcceleration) {
        super(
                () -> {
                    PathConstraints constraints =
                            new PathConstraints(velocity, acceleration, angularVelocity, angularAcceleration);
                    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
                },
                Set.of(swerve));
    }
}
