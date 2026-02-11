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
        // Back Left Camera
        new CameraConfig(
                "OV9782_1",
                new Transform3d(
                        Inches.of(-7.9),
                        Inches.of(9.75),
                        Inches.of(16.75),
                        new Rotation3d(Degrees.zero(), Degrees.of(0.0), Degrees.of(165.0))),
                1.0),
        // Back Right Camera
        new CameraConfig(
                "OV9782_2",
                new Transform3d(
                        Inches.of(-7.9),
                        Inches.of(-9.75),
                        Inches.of(16.75),
                        new Rotation3d(Degrees.zero(), Degrees.of(0.0), Degrees.of(195.0))),
                1.0),
        // Front Camera
        // new CameraConfig(
        //         "OV9782_3",
        //         new Transform3d(
        //                 Inches.of(0.0),
        //                 Inches.of(0.0),
        //                 Inches.of(0.0),
        //                 new Rotation3d(Degrees.zero(), Degrees.of(-15.0), Degrees.zero())),
        //         1.0),
    };

    public static final double kMaxAmbiguity = 0.30;
    public static final double kMaxZError = 0.75;

    // Lower values mean the estimator trusts the vision more.
    // Higher values mean the estimator trusts the vision less, relying more on encoders/gyro.
    // Values too low will rapidly "jitter", while too high values update slowly but ignore inaccurate instances.
    public static final double kLinearStdDevBaseline = 0.02; // Meters.
    public static final double kAngularStdDevBaseline = 0.06; // Radians.
}
