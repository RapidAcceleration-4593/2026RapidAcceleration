package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;

public final class Constants {

    public static enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode kCurrentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

    public static final Pose2d kBlueHubPose = new Pose2d(Inches.of(182.1), Inches.of(158.85), new Rotation2d());
    public static final Pose2d kRedHubPose = new Pose2d(Inches.of(469.1), Inches.of(158.85), new Rotation2d());

    public static final class Controllers {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }
}
