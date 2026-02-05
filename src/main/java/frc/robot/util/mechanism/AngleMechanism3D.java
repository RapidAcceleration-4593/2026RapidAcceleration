package frc.robot.util.mechanism;

import edu.wpi.first.units.measure.Angle;

public final class AngleMechanism3D extends SimpleMechanism3D {

    /**
     * Creates a new {@link AngleMechanism3D} which will set the rotation of the given {@link Node3D} along the given axis.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link AngleMechanism3D}.
     * @param axis The axis, in object space along which the {@link Node3D} should be
     *     rotated.
     */
    public AngleMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    /**
     * Creates a new {@link AngleMechanism3D} which will set the rotation of the given {@link Node3D} along the given axis.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link AngleMechanism3D}.
     * @param axis The axis, in object space along which the {@link Node3D} should be
     *     rotated.
	 * @param inverted If true, the angle input to {@link #setAngle()} will be multiplied by -1 before being applied to the {@link Node3D}.
     */
    public AngleMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
    }

    /**
     * Sets the angle of this {@link AngleMechanism3D}, changing the orientation of it's {@link Node3D} on the
     * visualization of the robot.
     *
     * @param angle The angle that the visualization should be set to.
     */
    public void setAngle(Angle angle) {
        node.setAngle(angle, axis, inverted);
    }
}
