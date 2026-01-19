package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class ObjectDetectionIOReal implements ObjectDetectionIO {

    private final PhotonCamera camera;

    public ObjectDetectionIOReal(String name) {
        camera = new PhotonCamera(name);
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        inputs.connected = camera.isConnected();

        List<TargetObservation> targets = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            if (result.hasTargets()) {
                for (PhotonTrackedTarget target : result.targets) {
                    targets.add(new TargetObservation(
                            Rotation2d.fromDegrees(target.getYaw()), Rotation2d.fromDegrees(target.getPitch())));
                }
            }
        }

        inputs.latestTargets = targets.toArray(new TargetObservation[0]);
    }
}
