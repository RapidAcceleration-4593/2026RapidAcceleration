package frc.robot.subsystems.vision.objectdetection;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class ObjectDetectionConstants {

    public record CameraConfig(String name, Transform3d robotToCamera) {}

    public static final CameraConfig[] kCameras = {
        new CameraConfig(
                "OV9782_4",
                new Transform3d(
                        Inches.of(10.0),
                        Inches.zero(),
                        Inches.of(20.0),
                        new Rotation3d(Degrees.zero(), Degrees.of(15.0), Degrees.zero()))),
    };
}