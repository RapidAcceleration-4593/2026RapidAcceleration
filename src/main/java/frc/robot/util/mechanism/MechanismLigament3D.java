package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class MechanismLigament3D {
    private Axis axis;
    private boolean inverted = false;
    private Node3D node;

    public MechanismLigament3D(Node3D node, Axis axis) {
        this.node = node;
        this.axis = axis;
    }

    public MechanismLigament3D(Node3D node, Axis axis, boolean inverted) {
        this.node = node;
        this.axis = axis;
        this.inverted = inverted;
    }

    public void setAngle(Angle angle) {
        switch (axis) {
            case X:
                node.setRelativePose(
                        new Pose3d(0, 0, 0, new Rotation3d(angle.in(Radians) * (inverted ? -1 : 1), 0, 0)));
                break;
            case Y:
                node.setRelativePose(
                        new Pose3d(0, 0, 0, new Rotation3d(0, angle.in(Radians) * (inverted ? -1 : 1), 0)));
                break;
            case Z:
                node.setRelativePose(
                        new Pose3d(0, 0, 0, new Rotation3d(0, 0, angle.in(Radians) * (inverted ? -1 : 1))));
                break;
        }
    }

    public void setDistance(Distance distance) {
        switch (axis) {
            case X:
                node.setRelativePose(new Pose3d(distance.in(Meters) * (inverted ? -1 : 1), 0, 0, Rotation3d.kZero));
                break;
            case Y:
                node.setRelativePose(new Pose3d(0, distance.in(Meters) * (inverted ? -1 : 1), 0, Rotation3d.kZero));
                break;
            case Z:
                node.setRelativePose(new Pose3d(0, 0, distance.in(Meters) * (inverted ? -1 : 1), Rotation3d.kZero));
                break;
        }
    }

    public String getName() {
        return node.getName();
    }

    Runnable getUpdateRunnable() {
        return node.getRootNode()::update;
    }

    Pose3d getPose3d() {
        return node.getGlobalPose();
    }
}
