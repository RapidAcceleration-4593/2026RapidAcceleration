package frc.robot.commands.swerve;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.FieldZones;
import java.util.Set;

public final class PathfindCommands {

    private static final PathConstraints constraints =
            new PathConstraints(kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);

    private static final Pose2d kCenterPose = new Pose2d(
            FieldUtil.getFieldLength().div(2), FieldUtil.getFieldWidth().div(2), new Rotation2d());
    private static final Pose2d kBluePose = new Pose2d(Inches.of(118.11), Inches.of(158.85), new Rotation2d());
    private static final Pose2d kRedPose =
            new Pose2d(Inches.of(533.09), Inches.of(158.85), Rotation2d.fromDegrees(180));

    public Command pathfindToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        return Commands.defer(
                () -> {
                    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
                },
                Set.of(swerve));
    }

    public Command pathfindToOppositeZone(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> {
                    Pose2d targetPose = getTargetPose(swerve.getPose());
                    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
                },
                Set.of(swerve));
    }

    private static FieldZones getTargetZone(Pose2d pose) {
        Alliance alliance = FieldUtil.getCurrentAlliance();
        FieldZones currentZone = FieldUtil.getCurrentZone(pose);

        if (alliance == Alliance.Blue) {
            return switch (currentZone) {
                case Blue_Zone, Red_Zone -> FieldZones.Neutral_Zone;
                case Neutral_Zone -> FieldZones.Blue_Zone;
            };
        } else {
            return switch (currentZone) {
                case Red_Zone, Blue_Zone -> FieldZones.Neutral_Zone;
                case Neutral_Zone -> FieldZones.Red_Zone;
            };
        }
    }

    private static Pose2d getTargetPose(Pose2d pose) {
        return switch (getTargetZone(pose)) {
            case Blue_Zone -> kBluePose;
            case Red_Zone -> kRedPose;
            case Neutral_Zone -> kCenterPose;
        };
    }
}
