package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

public class LinearMechanism3D extends Node3D {
    private Axis axis;
    private boolean inverted;

    public LinearMechanism3D(String name, Pose3d offset, Axis axis) {
        super(name, offset);
        this.axis = axis;
    }

    public LinearMechanism3D(String name, Pose3d offset, Axis axis, Node3D parent) {
        super(name, offset, parent);
        this.axis = axis;
    }

    public LinearMechanism3D(String name, Pose3d offset, Axis axis, boolean inverted) {
        super(name, offset);
        this.axis = axis;
        this.inverted = inverted;
    }

    public LinearMechanism3D(String name, Pose3d offset, Axis axis, Node3D parent, boolean inverted) {
        super(name, offset, parent);
        this.axis = axis;
        this.inverted = inverted;
    }

    public void setDistance(Distance distance) {
        switch (axis) {
            case X:
                this.relativePose = new Pose3d(distance.in(Meters) * (inverted ? -1 : 1), 0, 0, Rotation3d.kZero);
                break;
            case Y:
                this.relativePose = new Pose3d(0, distance.in(Meters) * (inverted ? -1 : 1), 0, Rotation3d.kZero);
                break;
            case Z:
                this.relativePose = new Pose3d(0, 0, distance.in(Meters) * (inverted ? -1 : 1), Rotation3d.kZero);
                break;
        }
    }
}
