package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;

public class AngularMechanism3D extends Node3D {
    private Axis axis;
    private boolean inverted;

    public AngularMechanism3D(String name, Pose3d offset, Axis axis) {
        super(name, offset);
        this.axis = axis;
    }

    public AngularMechanism3D(String name, Pose3d offset, Axis axis, Node3D parent) {
        super(name, offset, parent);
        this.axis = axis;
    }

    public AngularMechanism3D(String name, Pose3d offset, Axis axis, boolean inverted) {
        super(name, offset);
        this.axis = axis;
        this.inverted = inverted;
    }

    public AngularMechanism3D(String name, Pose3d offset, Axis axis, Node3D parent, boolean inverted) {
        super(name, offset, parent);
        this.axis = axis;
        this.inverted = inverted;
    }

    public void setAngle(Angle angle) {
        switch (axis) {
            case X:
                this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(angle.in(Radians) * (inverted ? -1 : 1), 0, 0));
                break;
            case Y:
                this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(0, angle.in(Radians) * (inverted ? -1 : 1), 0));
                break;
            case Z:
                this.relativePose = new Pose3d(0, 0, 0, new Rotation3d(0, 0, angle.in(Radians) * (inverted ? -1 : 1)));
                break;
        }
    }
}
