package frc.robot.commands.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

public class SimplePathCommand extends DeferredCommand {

    public SimplePathCommand(SwerveSubsystem swerve, Supplier<Pose2d> targetPoseSupplier) {
        super(
                () -> {
                    Pose2d startPose = swerve.getPose();
                    Pose2d endPose = targetPoseSupplier.get();
                    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, endPose);

                    PathConstraints constraints = new PathConstraints(
                            kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);
                    GoalEndState endState = new GoalEndState(0.0, endPose.getRotation());
                    PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, endState);

                    path.preventFlipping = true;
                    return AutoBuilder.followPath(null);
                },
                Set.of(swerve));
    }
}
