package frc.robot.commands.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;
import frc.robot.util.FieldUtil.FieldZones;
import java.util.Set;

public final class PathfindCommands {

    private static final PathConstraints kConstraints =
            new PathConstraints(kLinearVelocity, kLinearAcceleration, kAngularVelocity, kAngularAcceleration);

    public Command pathfindToPose(SwerveSubsystem swerve, Pose2d targetPose) {
        return Commands.defer(() -> AutoBuilder.pathfindToPose(targetPose, kConstraints, 0.0), Set.of(swerve));
    }

    public Command pathfindToOppositeZone(SwerveSubsystem swerve) {
        return Commands.defer(
                () -> AutoBuilder.pathfindToPose(getTargetPose(swerve.getPose()), kConstraints, 0.0), Set.of(swerve));
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
