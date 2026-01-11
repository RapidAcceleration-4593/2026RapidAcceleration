package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ObjectDetectionIOReal implements ObjectDetectionIO {

    private final NetworkTable table;
    private final NetworkTableEntry poseEntry;

    public ObjectDetectionIOReal() {
        table = NetworkTableInstance.getDefault().getTable("ObjectDetection");
        poseEntry = table.getEntry("objectPose");
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        boolean ntConnected = NetworkTableInstance.getDefault().isConnected();

        double[] poseArray = poseEntry.getDoubleArray(new double[0]);
        boolean validData = poseArray.length == 2 && !Double.isNaN(poseArray[0]) && !Double.isNaN(poseArray[1]);

        inputs.connected = ntConnected && validData;
        inputs.detectedPoses =
                validData ? new Pose2d[] {new Pose2d(poseArray[0], poseArray[1], new Rotation2d())} : new Pose2d[0];
    }
}
