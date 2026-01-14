package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class ObjectDetectionIOSim implements ObjectDetectionIO {

    private final Supplier<Pose2d> robotPoseSupplier;

    public ObjectDetectionIOSim(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Pose2d findClosestPose2d(List<Pose2d> poses) {
        Pose2d robotPose = robotPoseSupplier.get();
        return robotPose.nearest(poses);
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        List<Pose3d> objects3D = SimulatedArena.getInstance().getGamePiecesPosesByType("Coral");
        List<Pose2d> objects2D = new ArrayList<>();

        for (Pose3d pose3d : objects3D) {
            objects2D.add(pose3d.toPose2d());
        }

        inputs.connected = true;
        inputs.detectedPoses = new Pose2d[] {findClosestPose2d(objects2D)};
    }
}
