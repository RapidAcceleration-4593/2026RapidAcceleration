package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {

    @AutoLog
    public static class ObjectDetectionInputs {
        public boolean connected = false;
        public TargetObservation[] latestTargets = new TargetObservation[0];
    }

    /** Represents a single target's yaw/pitch from the camera. */
    public static record TargetObservation(Rotation2d yaw, Rotation2d pitch) {}

    public default void updateInputs(ObjectDetectionInputs inputs) {}
}
