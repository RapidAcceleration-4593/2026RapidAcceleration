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

    private record Trench(Pose2d allianceSide, Pose2d neutralSide) {}

    private record Bump(Pose2d allianceSide, Pose2d neutralSide) {}

    private static final List<Trench> kTrenches = List.of(
            new Trench(
                    new Pose2d(Meters.of(3.25), Meters.of(7.425), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(7.425), new Rotation2d())),
            new Trench(
                    new Pose2d(Meters.of(3.25), Meters.of(0.65), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(0.65), new Rotation2d())));

    private static final List<Bump> kBumps = List.of(
            new Bump(
                    new Pose2d(Meters.of(3.0), Meters.of(2.5), new Rotation2d()),
                    new Pose2d(Meter.of(6.0), Meters.of(2.5), new Rotation2d())),
            new Bump(
                    new Pose2d(Meters.of(3.0), Meters.of(5.5), new Rotation2d()),
                    new Pose2d(Meter.of(6.0), Meters.of(5.5), new Rotation2d())));

    public Command pathfindNearestTrench(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> {
                    Pose2d robotPose = swerve.getPose();
                    Trench trench = findClosestTrench(robotPose);

                    Pose2d entrance = getEntrance(robotPose, trench);
                    Pose2d exit = getExit(entrance, trench);

                    Rotation2d snapped = snapRotation(robotPose.getRotation());
                    Pose2d entranceWithRotation = new Pose2d(entrance.getTranslation(), snapped);
                    Pose2d exitWithRotation = new Pose2d(exit.getTranslation(), snapped);

                    return Commands.sequence(
                            AutoBuilder.pathfindToPose(entranceWithRotation, kConstraints, 2.0),
                            AutoBuilder.pathfindToPose(exitWithRotation, kConstraints, 0.0));
                },
                Set.of(swerve));
    }

    public Command pathfindNearestBump(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> {
                    Pose2d robotPose = swerve.getPose();
                    Bump bump = findClosestBump(robotPose);

                    Pose2d entrance = getEntrance(robotPose, bump);
                    Pose2d exit = getExit(entrance, bump);

                    Rotation2d snapped = snapRotation(robotPose.getRotation());
                    Pose2d entranceWithRotation = new Pose2d(entrance.getTranslation(), snapped);
                    Pose2d exitWithRotation = new Pose2d(exit.getTranslation(), snapped);

                    return Commands.sequence(
                            AutoBuilder.pathfindToPose(entranceWithRotation, kConstraints, 2.0),
                            new SimplePathCommand(swerve, exitWithRotation));
                },
                Set.of(swerve));
    }

    private Trench findClosestTrench(Pose2d robotPose) {
        return kTrenches.stream()
                .map(this::getCorrectTrench)
                .min((a, b) -> Double.compare(minDistanceTrench(robotPose, a), minDistanceTrench(robotPose, b)))
                .orElseThrow();
    }

    private Bump findClosestBump(Pose2d robotPose) {
        return kBumps.stream()
                .map(this::getCorrectBump)
                .min((a, b) -> Double.compare(minDistanceBump(robotPose, a), minDistanceBump(robotPose, b)))
                .orElseThrow();
    }

    private Trench getCorrectTrench(Trench blueTrench) {
        if (FieldUtil.isRedAlliance()) {
            return new Trench(
                    FlippingUtil.flipFieldPose(blueTrench.allianceSide()),
                    FlippingUtil.flipFieldPose(blueTrench.neutralSide()));
        }
        return blueTrench;
    }

    private Bump getCorrectBump(Bump blueBump) {
        if (FieldUtil.isRedAlliance()) {
            return new Bump(
                    FlippingUtil.flipFieldPose(blueBump.allianceSide()),
                    FlippingUtil.flipFieldPose(blueBump.neutralSide()));
        }
        return blueBump;
    }

    private double minDistanceTrench(Pose2d robotPose, Trench trench) {
        double d1 = robotPose.getTranslation().getDistance(trench.allianceSide().getTranslation());
        double d2 = robotPose.getTranslation().getDistance(trench.neutralSide().getTranslation());
        return Math.min(d1, d2);
    }

    private double minDistanceBump(Pose2d robotPose, Bump bump) {
        double d1 = robotPose.getTranslation().getDistance(bump.allianceSide().getTranslation());
        double d2 = robotPose.getTranslation().getDistance(bump.neutralSide().getTranslation());
        return Math.min(d1, d2);
    }

    private Pose2d getEntrance(Pose2d robotPose, Trench trench) {
        return robotPose.nearest(List.of(trench.allianceSide(), trench.neutralSide()));
    }

    private Pose2d getEntrance(Pose2d robotPose, Bump bump) {
        return robotPose.nearest(List.of(bump.allianceSide(), bump.neutralSide()));
    }

    private Pose2d getExit(Pose2d entrance, Trench trench) {
        return entrance.equals(trench.allianceSide()) ? trench.neutralSide() : trench.allianceSide();
    }

    private Pose2d getExit(Pose2d entrance, Bump bump) {
        return entrance.equals(bump.allianceSide()) ? bump.neutralSide() : bump.allianceSide();
    }

    private Rotation2d snapRotation(Rotation2d robotRotation) {
        return Rotation2d.fromDegrees(Math.round(robotRotation.getDegrees() / 180.0) * 180.0);
    }
}
