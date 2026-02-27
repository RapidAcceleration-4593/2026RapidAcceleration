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

    private static final List<Pose2d> kAllTrenchPoses = List.of(
            kLeftBlueAlliance,
            kLeftBlueNeutral,
            kRightBlueAlliance,
            kRightBlueNeutral,
            kLeftRedAlliance,
            kLeftRedNeutral,
            kRightRedAlliance,
            kRightRedNeutral);
    private static final List<Pose2d> kBlueTrenchPoses =
            List.of(kLeftBlueAlliance, kLeftBlueNeutral, kRightBlueAlliance, kRightBlueNeutral);
    private static final List<Pose2d> kRedTrenchPoses =
            List.of(kLeftRedAlliance, kLeftRedNeutral, kRightRedAlliance, kRightRedNeutral);

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
                    Pose2d initialPose = findClosest(swerve.getPose());
                    Pose2d finalPose = findNext(initialPose);
                    return Commands.sequence(
                            AutoBuilder.pathfindToPose(initialPose, kConstraints, 0.0),
                            AutoBuilder.pathfindToPose(finalPose, kConstraints, 0.0));
                },
                Set.of(swerve));
    }

    private Pose2d findClosest(Pose2d robotPose) {
        FieldZones currentZone = FieldUtil.getCurrentZone(robotPose);
        boolean isRedAlliance = FieldUtil.isRedAlliance();

        return switch (currentZone) {
            case Red_Zone -> robotPose.nearest(kRedTrenchPoses);
            case Blue_Zone -> robotPose.nearest(kBlueTrenchPoses);
            case Neutral_Zone ->
                isRedAlliance ? robotPose.nearest(kRedTrenchPoses) : robotPose.nearest(kBlueTrenchPoses);
        };
    }

    private Pose2d findNext(Pose2d firstPose) {
        return firstPose.nearest(kAllTrenchPoses); // Remove the firstPose.
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
