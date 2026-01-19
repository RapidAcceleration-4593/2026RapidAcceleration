package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class ObjectDetectionConstants {

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static final CameraConfig[] kCameras = {
        new CameraConfig(
                "Arducam_OV9782_Colored_4",
                new Transform3d(
                        Units.inchesToMeters(10.0),
                        Units.inchesToMeters(0.0),
                        Units.inchesToMeters(20.0),
                        new Rotation3d(0.0, Units.degreesToRadians(15), 0.0))),
    };
}
