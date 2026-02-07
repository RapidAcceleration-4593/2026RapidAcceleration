package frc.robot.util.mechanism;

import edu.wpi.first.math.geometry.Pose3d;

/**
 * Instances of this class as a mediator between the {@link Robot3D} and subsystems, ensuring that subsystems do not
 * have access to implementation details.
 *
 * <p>This class does not define any subsystem facing API as it is expected that such will vary widely between
 * implementations.
 */
public abstract class Mechanism3D {
    /**
     * Returns the name of the mechanism, which allows subsystems to find the specific mechanism it should be writing
     * to.
     *
     * @return The name of the mechanism.
     */
    protected abstract String getName();

    /**
     * Returns the {@link Runnable} that should be periodically called to update the return value of
     * {@link #getPose3d()}.
     *
     * @return A runnable that should be periodically called to update this {@link Mechanism3D}.
     */
    protected abstract Runnable getUpdateRunnable();

    /**
     * Returns the robot space position and rotation of this {@link Mechanism3D}.
     *
     * @return The pose, in robot space, of this {@link Mechanism3D}.
     */
    protected abstract Pose3d getPose3d();
}
