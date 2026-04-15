package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;
import java.util.List;
import java.util.Set;

public final class PathfindCommands {

    private static final PathConstraints kConstraints =
            new PathConstraints(kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);

    private record FieldObstacle(Pose2d allianceSide, Pose2d neutralSide) {
        public List<Pose2d> poses() {
            return List.of(allianceSide, neutralSide);
        }
    }

    private static final List<FieldObstacle> kTrenches = List.of(
            new FieldObstacle(
                    new Pose2d(Meters.of(3.25), Meters.of(7.425), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(7.425), new Rotation2d())),
            new FieldObstacle(
                    new Pose2d(Meters.of(3.25), Meters.of(0.65), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(0.65), new Rotation2d())));

    private static final List<FieldObstacle> kBumps = List.of(
            new FieldObstacle(
                    new Pose2d(Meters.of(3.0), Meters.of(2.5), new Rotation2d()),
                    new Pose2d(Meter.of(6.0), Meters.of(2.5), new Rotation2d())),
            new FieldObstacle(
                    new Pose2d(Meters.of(3.0), Meters.of(5.5), new Rotation2d()),
                    new Pose2d(Meter.of(6.0), Meters.of(5.5), new Rotation2d())));

    public Command pathfindNearestTrench(SwerveSubsystem swerve) {
        return pathfindObstacle(swerve, kTrenches);
    }

    public Command pathfindNearestBump(SwerveSubsystem swerve) {
        return pathfindObstacle(swerve, kBumps);
    }

    public Command pathfindObstacle(SwerveSubsystem swerve, List<FieldObstacle> obstacles) {
        return Commands.defer(
                () -> {
                    Pose2d robotPose = swerve.getPose();
                    FieldObstacle obstacle = obstacles.stream()
                            .map(this::getCorrectObstacle)
                            .min((a, b) -> Double.compare(minDistance(robotPose, a), minDistance(robotPose, b)))
                            .orElseThrow();

                    Pose2d entrance = robotPose.nearest(obstacle.poses());
                    Pose2d exit =
                            entrance.equals(obstacle.allianceSide()) ? obstacle.neutralSide() : obstacle.allianceSide();

                    Rotation2d snapped = Rotation2d.fromDegrees(
                            Math.round(robotPose.getRotation().getDegrees() / 180.0) * 180.0);
                    Pose2d entranceWithRotation = new Pose2d(entrance.getTranslation(), snapped);
                    Pose2d exitWithRotation = new Pose2d(exit.getTranslation(), snapped);

                    return Commands.sequence(
                            AutoBuilder.pathfindToPose(entranceWithRotation, kConstraints, 2.0),
                            new SimplePathCommand(swerve, exitWithRotation));
                },
                Set.of(swerve));
    }

    private FieldObstacle getCorrectObstacle(FieldObstacle blueObstacle) {
        if (FieldUtil.isRedAlliance()) {
            return new FieldObstacle(
                    FlippingUtil.flipFieldPose(blueObstacle.allianceSide()),
                    FlippingUtil.flipFieldPose(blueObstacle.neutralSide()));
        }
        return blueObstacle;
    }

    private double minDistance(Pose2d robotPose, FieldObstacle obstacle) {
        double d1 =
                robotPose.getTranslation().getDistance(obstacle.allianceSide().getTranslation());
        double d2 =
                robotPose.getTranslation().getDistance(obstacle.neutralSide().getTranslation());
        return Math.min(d1, d2);
    }
}
