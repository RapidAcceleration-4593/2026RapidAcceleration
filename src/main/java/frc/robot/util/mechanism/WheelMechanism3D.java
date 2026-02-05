package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public final class WheelMechanism3D extends SimpleMechanism3D {
    private double prevTimestamp;
    private AngularVelocity alpha;
    private double rotRadians;

    /**
     * Creates a new {@link WheelMechanism3D} which will continuously rotate {@link Node3D} along the given axis at the
     * set speed.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link AngleMechanism3D}.
     * @param axis The axis, in object space along which the {@link Node3D} should be spun.
     */
    public WheelMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    /**
     * Creates a new {@link WheelMechanism3D} which will continuously rotate {@link Node3D} along the given axis at the
     * set speed.
     *
     * @param node The {@link Node3D} whose pose should be updated by this {@link AngleMechanism3D}.
     * @param axis The axis, in object space along which the {@link Node3D} should be spun.
     * @param inverted If true, the input to {@link #setAngularVelocity()} will be multiplied by -1 before being applied
     *     to the {@link Node3D}.
     */
    public WheelMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
        prevTimestamp = Timer.getTimestamp();
        alpha = RadiansPerSecond.zero();
    }

    /**
     * Sets the angular velocity of this {@link AngleMechanism3D}, changing the orientation of it's {@link Node3D} on
     * the visualization of the robot.
     *
     * @param angle The rate at which the angle of the visualization should change.
     */
    public void setAngularVelocity(AngularVelocity alpha) {
        this.alpha = alpha;
    }

    protected void update() {
        var timestamp = Timer.getTimestamp();
        var deltaTime = timestamp - prevTimestamp;
        rotRadians += alpha.in(RadiansPerSecond) * deltaTime;
        prevTimestamp = timestamp;
        rotRadians = rotRadians % (2 * Math.PI);
        node.setAngle(Radians.of(rotRadians), axis, inverted);
        super.getUpdateRunnable().run();
    }

    @Override
    protected Runnable getUpdateRunnable() {
        return this::update;
    }
}
