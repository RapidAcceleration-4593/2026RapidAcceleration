package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;

public abstract class SimpleMechanism3D extends Mechanism3D {
    protected Axis axis;
    protected boolean inverted = false;
    protected Node3D node;

    public SimpleMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    public SimpleMechanism3D(Node3D node, Axis axis, boolean inverted) {
        this.node = node;
        this.axis = axis;
        this.inverted = inverted;
    }

    @Override
    protected String getName() {
        return node.getName();
    }

    @Override
    protected Runnable getUpdateRunnable() {
        return node.getRootNode()::update;
    }

    @Override
    protected Pose3d getPose3d() {
        return node.getGlobalPose();
    }
}
