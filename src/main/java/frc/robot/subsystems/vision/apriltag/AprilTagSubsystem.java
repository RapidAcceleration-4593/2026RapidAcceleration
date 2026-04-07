package frc.robot.subsystems.vision.apriltag;

import static frc.robot.subsystems.vision.VisionConstants.*;
import static frc.robot.subsystems.vision.apriltag.AprilTagConstants.*;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.VisionConsumer;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkString;

public class AprilTagSubsystem extends SubsystemBase {

    private final LoggedNetworkString selectedVisionSystem = new LoggedNetworkString(kSelectedVisionNTAddress);
    private final VisionConsumer consumer;
    private final VisionInputsAutoLogged[] inputs;
    private final Alert[] disconnectedAlerts;
    private final AprilTagIO[] io;

    public AprilTagSubsystem(VisionConsumer consumer, AprilTagIO[] io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize Inputs.
        this.inputs = new VisionInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionInputsAutoLogged();
        }

        // Initialize Alerts.
        this.disconnectedAlerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnectedAlerts[i] =
                    new Alert("Vision Camera " + kCameras[i].name() + " Disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target.
     *
     * @param cameraIndex The index of the camera to use.
     */
    public Rotation2d getTargetX(int cameraIndex) {
        return inputs[cameraIndex].latestTargetObservation.tx();
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/" + kCameras[i].name(), inputs[i]);
        }

        // Initialize Logger Values.
        List<Pose3d> allTagPoses = new LinkedList<>();
        List<Pose3d> allRobotPoses = new LinkedList<>();
        List<Pose3d> allRobotPosesAccepted = new LinkedList<>();
        List<Pose3d> allRobotPosesRejected = new LinkedList<>();

        // Loop Over Cameras.
        for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
            // Update Disconnected Alert.
            disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected);

            // Initialize Logging Values.
            List<Pose3d> tagPoses = new LinkedList<>();
            List<Pose3d> robotPoses = new LinkedList<>();
            List<Pose3d> robotPosesAccepted = new LinkedList<>();
            List<Pose3d> robotPosesRejected = new LinkedList<>();

            // Add AprilTag Poses
            for (int tagId : inputs[cameraIndex].tagIds) {
                var tagPose = kFieldLayout.getTagPose(tagId);
                if (tagPose.isPresent()) {
                    tagPoses.add(tagPose.get());
                }
            }

            // Loop through Pose Observations.
            for (var observation : inputs[cameraIndex].poseObservations) {
                // Check whether to reject each pose.
                boolean rejectPose = observation.tagCount() == 0 // Must have at least one tag.
                        || (observation.tagCount() == 1
                                && observation.ambiguity() > kMaxAmbiguity) // Cannot be high ambiguity.
                        || Math.abs(observation.pose().getZ()) > kMaxZError // Must have a realistic Z coordinate.
                        || observation.pose().getX() < 0.0
                        || observation.pose().getX() > kFieldLayout.getFieldLength()
                        || observation.pose().getY() < 0.0
                        || observation.pose().getY() > kFieldLayout.getFieldWidth();

                // Add Pose to Log.
                robotPoses.add(observation.pose());
                if (rejectPose) {
                    robotPosesRejected.add(observation.pose());
                } else {
                    robotPosesAccepted.add(observation.pose());
                }

                // Skip Rejected Poses.
                if (rejectPose) {
                    continue;
                }

                // Calculate Standard Deviations.
                double stdDevFactor = Math.pow(observation.averageTagDistance(), 2.0) / observation.tagCount();
                double linearStdDev = kLinearStdDevBaseline * stdDevFactor;
                double angularStdDev = kAngularStdDevBaseline * stdDevFactor;

                double cameraStdDevFactor = kCameras[cameraIndex].stdDevFactor();
                linearStdDev *= cameraStdDevFactor;
                angularStdDev *= cameraStdDevFactor;

                // If this is the selected vision system, send pose observation
                if (selectedVisionSystem.get().equals(VisionSystems.Apriltag.name)) {
                    consumer.accept(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
                }
            }

            String cameraName = kCameras[cameraIndex].name();

            // Log Camera Data.
            Logger.recordOutput("Vision/" + cameraName + "/TagPoses", tagPoses.toArray(new Pose3d[tagPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + cameraName + "/RobotPoses", robotPoses.toArray(new Pose3d[robotPoses.size()]));
            Logger.recordOutput(
                    "Vision/" + cameraName + "/RobotPosesAccepted",
                    robotPosesAccepted.toArray(new Pose3d[robotPosesAccepted.size()]));
            Logger.recordOutput(
                    "Vision/" + cameraName + "/RobotPosesRejected",
                    robotPosesRejected.toArray(new Pose3d[robotPosesRejected.size()]));

            allTagPoses.addAll(tagPoses);
            allRobotPoses.addAll(robotPoses);
            allRobotPosesAccepted.addAll(robotPosesAccepted);
            allRobotPosesRejected.addAll(robotPosesRejected);
        }

        // Log Summary Data.
        Logger.recordOutput("Vision/Summary/TagPoses", allTagPoses.toArray(new Pose3d[allTagPoses.size()]));
        Logger.recordOutput("Vision/Summary/RobotPoses", allRobotPoses.toArray(new Pose3d[allRobotPoses.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesAccepted",
                allRobotPosesAccepted.toArray(new Pose3d[allRobotPosesAccepted.size()]));
        Logger.recordOutput(
                "Vision/Summary/RobotPosesRejected",
                allRobotPosesRejected.toArray(new Pose3d[allRobotPosesRejected.size()]));
    }
}
