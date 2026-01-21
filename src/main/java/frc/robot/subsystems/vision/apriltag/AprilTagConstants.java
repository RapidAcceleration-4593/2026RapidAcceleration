package frc.robot.subsystems.vision.apriltag;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class AprilTagConstants {

    public static final AprilTagFieldLayout kFieldLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    public record CameraConfig(String name, Transform3d robotToCamera, double stdDevFactor) {}

    public static final CameraConfig[] kCameras = {
        // Front Camera
        new CameraConfig(
                "OV9782_1",
                new Transform3d(
                        Inches.of(5.0),
                        Inches.of(0.0),
                        Inches.of(20.0),
                        new Rotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.zero())),
                1.0),
        // Back Left Camera
        new CameraConfig(
                "OV9782_2",
                new Transform3d(
                        Inches.of(-8.0),
                        Inches.of(6.0),
                        Inches.of(16.0),
                        new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(165.0))),
                1.0),
        // Back Right Camera
        new CameraConfig(
                "OV9782_3",
                new Transform3d(
                        Inches.of(-8.0),
                        Inches.of(-6.0),
                        Inches.of(16.0),
                        new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(195.0))),
                1.0)
    };

    public static final double kMaxAmbiguity = 0.30;
    public static final double kMaxZError = 0.75;

    public static final double kLinearStdDevBaseline = 0.02; // Meters.
    public static final double kAngularStdDevBaseline = 0.06; // Radians.
}
