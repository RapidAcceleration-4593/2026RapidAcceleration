package frc.robot.util.mechanism;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;

public final class WheelMechanism3D extends SimpleMechanism3D {
    private double prevTimestamp;
    private AngularVelocity alpha;
    private double rotRadians;

    public WheelMechanism3D(Node3D node, Axis axis) {
        this(node, axis, false);
    }

    public WheelMechanism3D(Node3D node, Axis axis, boolean inverted) {
        super(node, axis, inverted);
        prevTimestamp = Timer.getTimestamp();
        alpha = RadiansPerSecond.zero();
    }

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
