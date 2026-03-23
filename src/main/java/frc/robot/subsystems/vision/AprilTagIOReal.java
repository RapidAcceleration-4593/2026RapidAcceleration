package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.AprilTagConstants.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import org.photonvision.PhotonCamera;

public class AprilTagIOReal implements AprilTagIO {

    protected final PhotonCamera camera;
    protected final Transform3d robotToCamera;

    public AprilTagIOReal(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    private Pose3d computePose(Transform3d fieldToCamera) {
        Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
        return new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
    }

    @Override
    public void updateInputs(VisionInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read New Camera Observations.
        Set<Short> tagIds = new HashSet<>();
        List<PoseObservation> poseObservations = new LinkedList<>();

        for (var result : camera.getAllUnreadResults()) {
            // Update Latest Target Observations.
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(
                        Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                        Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            }

            // Add Pose Observation.
            if (result.multitagResult.isPresent()) { // Multitag Result.
                var multitagResult = result.multitagResult.get();

                // Calculate Robot Pose.
                Pose3d robotPose = computePose(multitagResult.estimatedPose.best);

                // Calculate Average AprilTag Distance.
                double totalTagDistance = 0.0;
                for (var target : result.targets) {
                    totalTagDistance +=
                            target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add AprilTag IDs.
                tagIds.addAll(multitagResult.fiducialIDsUsed);

                // Add Observation.
                poseObservations.add(new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp.
                        robotPose, // 3D Pose Estimate.
                        multitagResult.estimatedPose.ambiguity, // Ambiguity.
                        multitagResult.fiducialIDsUsed.size(), // AprilTag Count.
                        totalTagDistance / result.targets.size() // Average AprilTag Distance.
                        ));
            } else if (!result.targets.isEmpty()) { // Single AprilTag Result.
                var target = result.targets.get(0);

                // Calculate Robot Pose.
                var tagPose = kFieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(
                            tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Pose3d robotPose = computePose(fieldToCamera);

                    // Add AprilTag ID.
                    tagIds.add((short) target.fiducialId);

                    poseObservations.add(new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp.
                            robotPose, // 3D Pose Estimate.
                            target.poseAmbiguity, // Ambiguity.
                            1, // AprilTag Count.
                            cameraToTarget.getTranslation().getNorm() // Average AprilTag Distance.
                            ));
                }
            }
        }

        // Save Pose Observations to Inputs Object.
        inputs.poseObservations = poseObservations.toArray(new PoseObservation[0]);

        // Save AprilTag IDs to Inputs Object.
        inputs.tagIds = tagIds.stream().mapToInt(Short::intValue).toArray();
    }
}
