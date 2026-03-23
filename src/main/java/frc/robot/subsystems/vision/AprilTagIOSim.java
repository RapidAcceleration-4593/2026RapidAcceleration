package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.SimulationManager;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;

public class AprilTagIOSim extends AprilTagIOReal {

    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new SimVisionIO for PhotonVision.
     *
     * @param name The name of the camera.
     * @param poseSupplier A supplier that provides the robot pose to use in simulation.
     */
    public AprilTagIOSim(String name, Transform3d robotToCamera, Supplier<Pose2d> poseSupplier) {
        super(name, robotToCamera);

        // Add Simulation Camera.
        var cameraProperties = new SimCameraProperties();
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        SimulationManager.getInstance().getVisionSim().addCamera(cameraSim, robotToCamera);
    }
}
