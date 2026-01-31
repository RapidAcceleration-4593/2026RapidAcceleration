package frc.robot.util.mechanism;

import edu.wpi.first.units.measure.Angle;

public class AngleMechanism3D extends SimpleMechanism3D {

    public AngleMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    public AngleMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
    }

    public void setAngle(Angle angle) {
        node.setAngle(angle, axis, inverted);
    }
}
