package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtil {

    public static final Distance kFieldLength = Inches.of(651.2);
    public static final Distance kFieldWidth = Inches.of(317.7);

    private static final Pose3d kBlueHubPose =
            new Pose3d(Inches.of(182.1), kFieldWidth.div(2), Inches.of(72.0), new Rotation3d());
    private static final Pose3d kRedHubPose =
            new Pose3d(Inches.of(469.1), kFieldWidth.div(2), Inches.of(72.0), new Rotation3d());
    private static final Pose3d kBlueCFFPose =
            new Pose3d(kFieldLength.times(0.15), kFieldWidth.div(2), Inches.zero(), new Rotation3d());
    private static final Pose3d kRedCFFPose =
            new Pose3d(kFieldLength.times(0.85), kFieldWidth.div(2), Inches.zero(), new Rotation3d());

    public enum FieldZones {
        Neutral_Zone,
        Blue_Zone,
        Red_Zone
    }

    public static Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static boolean isRedAlliance() {
        return getCurrentAlliance() == Alliance.Red;
    }

    public static Pose3d getTargetHubPose() {
        return getCurrentAlliance() == Alliance.Blue ? kBlueHubPose : kRedHubPose;
    }

    public static Pose3d getCrossFieldFeedPose() {
        return getCurrentAlliance() == Alliance.Blue ? kBlueCFFPose : kRedCFFPose;
    }

    public static Pose3d getTargetPose(Pose2d robotPose) {
        boolean isInAllianceZone = isInAllianceZone(robotPose);
        return isInAllianceZone ? getTargetHubPose() : getCrossFieldFeedPose();
    }

    public static FieldZones getCurrentZone(Pose2d robotPose) {
        if (robotPose.getMeasureX().lt(kBlueHubPose.getMeasureX())) return FieldZones.Blue_Zone;
        if (robotPose.getMeasureX().gt(kRedHubPose.getMeasureX())) return FieldZones.Red_Zone;
        return FieldZones.Neutral_Zone;
    }

    public static boolean isInAllianceZone(Pose2d robotPose) {
        FieldZones zone = getCurrentZone(robotPose);
        Alliance alliance = getCurrentAlliance();
        return (alliance == Alliance.Red && zone == FieldZones.Red_Zone)
                || (alliance == Alliance.Blue && zone == FieldZones.Blue_Zone);
    }

    public static Pose2d getInitialPose() {
        return isRedAlliance() ? kRedCFFPose.toPose2d() : kBlueCFFPose.toPose2d();
    }
}
