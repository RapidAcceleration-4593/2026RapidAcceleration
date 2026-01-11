package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public final class ObjectDetectionConstants {

    public static final Transform2d cameraTransform =
            new Transform2d(Units.inchesToMeters(10.0), Units.inchesToMeters(0.0), new Rotation2d());
}
