package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public final class FieldUtil {

    public static final Distance kFieldLength = Inches.of(651.2);
    public static final Distance kFieldWidth = Inches.of(317.7);

    private static final Pose3d kBlueHubPose =
            new Pose3d(Inches.of(182.1), kFieldWidth.div(2), Inches.of(72.0), Rotation3d.kZero);
    private static final Pose3d kRedHubPose =
            new Pose3d(Inches.of(469.1), kFieldWidth.div(2), Inches.of(72.0), Rotation3d.kZero);
    private static final Pose2d kBlueInitialPose =
            new Pose2d(kFieldLength.times(0.22), kFieldWidth.div(2), Rotation2d.kZero);
    private static final Pose2d kRedInitialPose =
            new Pose2d(kFieldLength.times(0.78), kFieldWidth.div(2), Rotation2d.k180deg);

    private static final Rectangle2d kBlueRightTrench =
            new Rectangle2d(new Translation2d(4.028, 0), new Translation2d(5.222, 1.718));
    private static final Rectangle2d kBlueLeftTrench =
            new Rectangle2d(new Translation2d(4.028, 8.070), new Translation2d(5.222, 6.351));
    private static final Rectangle2d kRedLeftTrench =
            new Rectangle2d(new Translation2d(12.512, 0), new Translation2d(11.318, 1.718));
    private static final Rectangle2d kRedRightTrench =
            new Rectangle2d(new Translation2d(12.512, 8.070), new Translation2d(11.318, 6.351));

    public enum FieldZones {
        Neutral_Zone,
        Blue_Zone,
        Red_Zone
    }

    public static Supplier<Pose2d> poseSupplier;

    public static void setPoseSupplier(Supplier<Pose2d> supplier) {
        poseSupplier = supplier;
    }

    private static Pose2d getPose() {
        return poseSupplier.get();
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
        Distance x = isRedAlliance() ? kFieldLength.times(0.9) : kFieldLength.times(0.1);
        Distance y =
                getPose().getMeasureY().gt(kFieldWidth.times(0.5)) ? kFieldWidth.times(0.8) : kFieldWidth.times(0.2);
        return new Pose3d(x, y, Inches.zero(), Rotation3d.kZero);
    }

    public static Pose3d getTargetPose() {
        if (DriverStation.isAutonomous()) return getTargetHubPose();

        boolean isInAllianceZone = isInAllianceZone();
        return isInAllianceZone ? getTargetHubPose() : getCrossFieldFeedPose();
    }

    public static FieldZones getCurrentZone() {
        Pose2d robotPose = getPose();
        if (robotPose.getMeasureX().lt(kBlueHubPose.getMeasureX())) return FieldZones.Blue_Zone;
        if (robotPose.getMeasureX().gt(kRedHubPose.getMeasureX())) return FieldZones.Red_Zone;
        return FieldZones.Neutral_Zone;
    }

    public static boolean isInAllianceZone() {
        FieldZones zone = getCurrentZone();
        Alliance alliance = getCurrentAlliance();
        return (alliance == Alliance.Red && zone == FieldZones.Red_Zone)
                || (alliance == Alliance.Blue && zone == FieldZones.Blue_Zone);
    }

    public static boolean isUnderTrench() {
        Translation2d shooterPos = getPose().transformBy(kPhysicalOffset).getTranslation();
        return kBlueLeftTrench.contains(shooterPos)
                || kBlueRightTrench.contains(shooterPos)
                || kRedRightTrench.contains(shooterPos)
                || kRedLeftTrench.contains(shooterPos);
    }

    public static Pose2d getInitialPose() {
        return isRedAlliance() ? kRedInitialPose : kBlueInitialPose;
    }
}
