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

		/*right now, my program requires getting inputs from a single point in time. this is already how the sim version does it, 
		though it's also COMPLETELY ACCURATE. However, I need some alternative method in the real version to do the same thing.*/
    }
}
