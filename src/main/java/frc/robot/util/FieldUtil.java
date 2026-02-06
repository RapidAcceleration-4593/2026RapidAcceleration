package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtil {

    private static final Distance kBlueAllianceBoundary = Inches.of(184.1);
    private static final Distance kRedAllianceBoundary = Inches.of(467.1);

    private static final Pose2d kInitialBluePose = new Pose2d(Inches.of(118.11), Inches.of(158.85), new Rotation2d());
    private static final Pose2d kInitialRedPose =
            new Pose2d(Inches.of(533.09), Inches.of(158.85), Rotation2d.fromDegrees(180));

    private static final Pose2d kBlueHubPose = new Pose2d(Inches.of(182.1), Inches.of(158.85), new Rotation2d());
    private static final Pose2d kRedHubPose = new Pose2d(Inches.of(469.1), Inches.of(158.85), new Rotation2d());

    public enum FieldZones {
        Neutral_Zone,
        Blue_Zone,
        Red_Zone
    }

    public static Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static Pose2d getInitialPose() {
        return getCurrentAlliance() == Alliance.Blue ? kInitialBluePose : kInitialRedPose;
    }

    public static Pose2d getTargetHubPose() {
        return getCurrentAlliance() == Alliance.Blue ? kBlueHubPose : kRedHubPose;
    }

    public static FieldZones getCurrentZone(Pose2d pose) {
        if (pose.getMeasureX().lt(kBlueAllianceBoundary)) return FieldZones.Blue_Zone;
        if (pose.getMeasureX().gt(kRedAllianceBoundary)) return FieldZones.Red_Zone;
        return FieldZones.Neutral_Zone;
    }

    public static Distance getFieldLength() {
        return Inches.of(651.2);
    }

    public static Distance getFieldWidth() {
        return Inches.of(317.7);
    }
}
