package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;

public abstract class Mechanism3D {
    protected abstract String getName();

    protected abstract Runnable getUpdateRunnable();

    protected abstract Pose3d getPose3d();
}
