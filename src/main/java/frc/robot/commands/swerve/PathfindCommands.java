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

    private static final PathConstraints kClimbConstants = new PathConstraints(
            MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(1.0), kAngularVelocity, kAngularAcceleration);

    private record Trench(Pose2d allianceSide, Pose2d neutralSide) {}

    private static final Pose2d kLeftClimb = new Pose2d(Meters.of(0.92), Meters.of(3.0), new Rotation2d());
    private static final Pose2d kRightClimb = new Pose2d(Meters.of(1.2), Meters.of(0.45), new Rotation2d());

    private static final List<Trench> kTrenches = List.of(
            new Trench(
                    new Pose2d(Meters.of(3.25), Meters.of(7.425), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(7.425), new Rotation2d())),
            new Trench(
                    new Pose2d(Meters.of(3.25), Meters.of(0.65), new Rotation2d()),
                    new Pose2d(Meters.of(6.0), Meters.of(0.65), new Rotation2d())));

    private Pose2d fieldPose(Pose2d bluePose) {
        return FieldUtil.isRedAlliance() ? FlippingUtil.flipFieldPose(bluePose) : bluePose;
    }

    public Command pathfindUnderNearestTrench(SwerveSubsystem swerve) {
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
                            AutoBuilder.pathfindToPose(entranceWithRotation, kConstraints, 1.0),
                            AutoBuilder.pathfindToPose(exitWithRotation, kConstraints, 0.0));
                },
                Set.of(swerve));
    }

    public Command pathfindLeftClimb(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(fieldPose(kLeftClimb), kClimbConstants, 0.0), Set.of(swerve));
    }

    public Command pathfindRightClimb(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(fieldPose(kRightClimb), kClimbConstants, 0.0), Set.of(swerve));
    }

    private Trench findClosestTrench(Pose2d robotPose) {
        return kTrenches.stream()
                .map(this::getCorrectTrench)
                .min((a, b) -> Double.compare(minDistance(robotPose, a), minDistance(robotPose, b)))
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

    private double minDistance(Pose2d robotPose, Trench trench) {
        double d1 = robotPose.getTranslation().getDistance(trench.allianceSide().getTranslation());
        double d2 = robotPose.getTranslation().getDistance(trench.neutralSide().getTranslation());
        return Math.min(d1, d2);
    }

    private Pose2d getEntrance(Pose2d robotPose, Trench trench) {
        return robotPose.nearest(List.of(trench.allianceSide(), trench.neutralSide()));
    }

    private Pose2d getExit(Pose2d entrance, Trench trench) {
        return entrance.equals(trench.allianceSide()) ? trench.neutralSide() : trench.allianceSide();
    }

    private Rotation2d snapRotation(Rotation2d robotRotation) {
        return Rotation2d.fromDegrees(Math.round(robotRotation.getDegrees() / 180.0) * 180.0);
    }
}
