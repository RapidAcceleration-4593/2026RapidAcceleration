package frc.robot.subsystems.vision.apriltag;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public final class AprilTagConstants {

    public static final AprilTagFieldLayout kFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}

    public static final CameraConfig[] kCameras = {
        // Left Camera
        new CameraConfig(
                "Arducam_OV9782_Colored_1",
                new Transform3d(
                        Units.inchesToMeters(10.0),
                        Units.inchesToMeters(-9.0),
                        Units.inchesToMeters(19.5),
                        new Rotation3d(0.0, Units.degreesToRadians(15), 0.0)),
                1.0),
        // Right Camera
        new CameraConfig(
                "Arducam_OV9782_Colored_2",
                new Transform3d(
                        Units.inchesToMeters(10.0),
                        Units.inchesToMeters(9.0),
                        Units.inchesToMeters(19.5),
                        new Rotation3d(0.0, Units.degreesToRadians(15), 0.0)),
                1.0)
    };

    public static final double kMaxAmbiguity = 0.30;
    public static final double kMaxZError = 0.75;

    public static final double kLinearStdDevBaseline = 0.02; // Meters.
    public static final double kAngularStdDevBaseline = 0.06; // Radians.
}
