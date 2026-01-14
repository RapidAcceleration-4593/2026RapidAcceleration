package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetectionIOReal implements ObjectDetectionIO {

    private final NetworkTable table;
    private final NetworkTableEntry densityEntry;

    public ObjectDetectionIOReal() {
        table = NetworkTableInstance.getDefault().getTable("ObjectDetection");
        densityEntry = table.getEntry("objectDensity");
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        boolean ntConnected = NetworkTableInstance.getDefault().isConnected();

        double[] density = densityEntry.getDoubleArray(new double[0]);
        boolean validData = density.length == 2 && !Double.isNaN(density[0]) && !Double.isNaN(density[1]);

        inputs.connected = ntConnected && validData;

        if (validData) {
            double xError = density[0]; // Positive right.
            double yError = density[1]; // Positive forward.
            Rotation2d rotationToTarget = new Rotation2d(Math.atan2(yError, xError));
            inputs.detectedPoses = new Pose2d[] {new Pose2d(xError, yError, rotationToTarget)};
        } else {
            inputs.detectedPoses = new Pose2d[0];
        }
    }
}
