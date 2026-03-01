package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.FieldZones;
import java.util.List;
import java.util.Set;

public final class PathfindCommands {

    private static final Pose2d kLeftBlueAlliance = new Pose2d(Meters.of(3.25), Meters.of(7.425), new Rotation2d());
    private static final Pose2d kLeftBlueNeutral = new Pose2d(Meters.of(6.0), Meters.of(7.425), new Rotation2d());

    private static final Pose2d kRightBlueAlliance = new Pose2d(Meters.of(3.25), Meters.of(0.65), new Rotation2d());
    private static final Pose2d kRightBlueNeutral = new Pose2d(Meters.of(6.0), Meters.of(0.65), new Rotation2d());

    private static final Pose2d kLeftRedAlliance = new Pose2d(Meters.of(13.33), Meters.of(0.65), new Rotation2d());
    private static final Pose2d kLeftRedNeutral = new Pose2d(Meters.of(10.5), Meters.of(0.65), new Rotation2d());

    private static final Pose2d kRightRedAlliance = new Pose2d(Meters.of(13.33), Meters.of(7.425), new Rotation2d());
    private static final Pose2d kRightRedNeutral = new Pose2d(Meters.of(10.5), Meters.of(7.425), new Rotation2d());

    private record Trench(Pose2d allianceSide, Pose2d neutralSide) {}

    private static final List<Trench> kBlueTrenches =
            List.of(new Trench(kLeftBlueAlliance, kLeftBlueNeutral), new Trench(kRightBlueAlliance, kRightBlueNeutral));

    private static final List<Trench> kRedTrenches =
            List.of(new Trench(kLeftRedAlliance, kLeftRedNeutral), new Trench(kRightRedAlliance, kRightRedNeutral));

    private static final PathConstraints kConstraints =
            new PathConstraints(kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);

    public Command pathfindToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        return Commands.defer(() -> AutoBuilder.pathfindToPose(targetPose, kConstraints, 0.0), Set.of(swerve));
    }

    public Command pathfindToOppositeZone(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(getTargetPose(swerve.getPose()), kConstraints, 0.0), Set.of(swerve));
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
                            AutoBuilder.pathfindToPose(exitWithRotation, kConstraints, 1.0));
                },
                Set.of(swerve));
    }

    private Trench findClosestTrench(Pose2d robotPose) {
        List<Trench> trenches = FieldUtil.isRedAlliance() ? kRedTrenches : kBlueTrenches;

        return trenches.stream()
                .min((a, b) -> {
                    double da = minDistance(robotPose, a);
                    double db = minDistance(robotPose, b);
                    return Double.compare(da, db);
                })
                .orElseThrow();
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
        double angle = robotRotation.getRadians();
        double wrapped = Math.IEEEremainder(angle, Math.PI);

        return Math.abs(wrapped) < Math.PI / 2 ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180);
    }

    private static FieldZones getTargetZone(Pose2d currentPose) {
        FieldZones currentZone = FieldUtil.getCurrentZone(currentPose);
        boolean isRedAlliance = FieldUtil.isRedAlliance();

        return switch (currentZone) {
            case Red_Zone, Blue_Zone -> FieldZones.Neutral_Zone;
            case Neutral_Zone -> isRedAlliance ? FieldZones.Red_Zone : FieldZones.Blue_Zone;
        };
    }

    private static Pose2d getTargetPose(Pose2d currentPose) {
        FieldZones targetZone = getTargetZone(currentPose);

        return switch (targetZone) {
            case Blue_Zone -> FieldUtil.kInitialBluePose;
            case Red_Zone -> FieldUtil.kInitialRedPose;
            case Neutral_Zone -> FieldUtil.kInitialCenterPose;
        };
    }
}
