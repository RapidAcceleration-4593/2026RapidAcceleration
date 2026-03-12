package frc.robot.commands.auton;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;

public class AutonUtil {

    private final SwerveSubsystem swerve;
    private final RobotConfig robotConfig;

    public AutonUtil(SwerveSubsystem swerve) {
        this.swerve = swerve;

        try {
            this.robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load robot config from GUI settings", e);
        }
    }

    public PathPlannerPath loadPath(String path) {
        try {
            return PathPlannerPath.fromPathFile(path);
        } catch (Exception e) {
            e.printStackTrace();
            throw new RuntimeException("Failed to load path: " + path, e);
        }
    }

    public Command resetOdometry(PathPlannerPath path) {
        return swerve.runOnce(() -> swerve.setPose(getStartingPose(path)));
    }

    public Pose2d getStartingPose(PathPlannerPath path) {
        return flipPose(path.getStartingHolonomicPose()
                .orElseThrow(() -> new RuntimeException("Path does not have a starting holonomic pose.")));
    }

    public Pose2d flipPose(Pose2d pose) {
        return FieldUtil.isRedAlliance() ? FlippingUtil.flipFieldPose(pose) : pose;
    }

    public RobotConfig getRobotConfig() {
        return robotConfig;
    }
}
