package frc.robot.util;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class FieldUtil {

    public static final Distance kFieldLength = Inches.of(651.2);
    public static final Distance kFieldWidth = Inches.of(317.7);

    private static final Pose3d kBlueHubPose =
            new Pose3d(Inches.of(182.1), kFieldWidth.div(2), Feet.of(10.0), Rotation3d.kZero);
    private static final Pose3d kRedHubPose =
            new Pose3d(Inches.of(469.1), kFieldWidth.div(2), Feet.of(10.0), Rotation3d.kZero);
    private static final Pose2d kBlueInitialPose =
            new Pose2d(kFieldLength.times(0.22), kFieldWidth.div(2), Rotation2d.kZero);
    private static final Pose2d kRedInitialPose =
            new Pose2d(kFieldLength.times(0.78), kFieldWidth.div(2), Rotation2d.k180deg);

    public static Alliance getCurrentAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Red);
    }

    public static boolean isRedAlliance() {
        return getCurrentAlliance() == Alliance.Red;
    }

    public static Pose3d getTargetPose() {
        return getCurrentAlliance() == Alliance.Blue ? kBlueHubPose : kRedHubPose;
    }

    public static Pose2d getInitialPose() {
        return isRedAlliance() ? kRedInitialPose : kBlueInitialPose;
    }
}
