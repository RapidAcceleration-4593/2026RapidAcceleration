package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface ObjectDetectionIO {

    @AutoLog
    public static class ObjectDetectionInputs {
        public boolean connected = false;
        public Pose2d[] detectedPoses = new Pose2d[0];
    }

    public default void updateInputs(ObjectDetectionInputs inputs) {}
}
