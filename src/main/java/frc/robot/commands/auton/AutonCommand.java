package frc.robot.commands.auton;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.List;

public abstract class AutonCommand extends SequentialCommandGroup {

    protected final AutonUtil util;
    protected final List<PathPlannerPath> paths;

    protected AutonCommand(AutonUtil util, List<String> pathNames) {
        this.util = util;
        this.paths = pathNames.stream().map(util::loadPath).toList();

        if (paths.isEmpty()) throw new IllegalArgumentException("At least one path must be provided.");

        addCommands(util.resetOdometry(paths.get(0)));
    }

    public List<Pose2d> getAllPathPoses() {
        return paths.stream()
                .map(PathPlannerPath::getPathPoses)
                .flatMap(List::stream)
                .map(util::flipPose)
                .toList();
    }

    public Pose2d getStartingPose() {
        return util.getStartingPose(paths.get(0));
    }

    public List<PathPlannerPath> getPaths() {
        return paths;
    }
}
