package frc.robot.util.mechanism;

import edu.wpi.first.units.measure.Distance;

public final class LengthMechanism3D extends SimpleMechanism3D {

    /**
     * Creates a new {@link LengthMechanism3D} which will set the translation of the given {@link Node3D} along the given axis, relative to its parent node.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link LengthMechanism3D}.
     * @param axis The axis, in object space, along which the {@link Node3D} should be
     *     rotated.
     */
    public LengthMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    /**
     * Creates a new {@link LengthMechanism3D} which will set the translation of the given {@link Node3D} along the given axis, relative to its parent node.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link LengthMechanism3D}.
     * @param axis The axis, in object space, along which the {@link Node3D} should be
     *     rotated.
	 * @param inverted If true, the angle input to {@link #setLength()} will be multiplied by -1 before being applied to the {@link Node3D}.
     */
    public LengthMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
    }

    /**
     * Sets the distance/length of this {@link AngleMechanism3D}, changing the orientation of it's {@link Node3D} on the
     * visualization of the robot.
     *
     * @param distance The distance that the visualization should be set to.
     */
    public void setLength(Distance distance) {
        node.setRelativeDistance(distance, axis, inverted);
    }
}
