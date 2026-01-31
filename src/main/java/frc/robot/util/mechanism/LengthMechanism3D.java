package frc.robot.util.mechanism;

import edu.wpi.first.units.measure.Distance;

public class LengthMechanism3D extends SimpleMechanism3D {

    public LengthMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    public LengthMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
    }

    public void setLength(Distance distance) {
        node.setRelativeDistance(distance, axis, inverted);
    }
}
