package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class ObjectDetectionIOSim implements ObjectDetectionIO {

    private final Supplier<Pose2d> robotPoseSupplier;
    private final int maxTargets;

    public ObjectDetectionIOSim(Supplier<Pose2d> robotPoseSupplier, int maxTargets) {
        this.robotPoseSupplier = robotPoseSupplier;
        this.maxTargets = maxTargets;
    }

    @Override
    public void updateInputs(ObjectDetectionInputs inputs) {
        inputs.connected = true;

        Pose2d robotPose = robotPoseSupplier.get();
        Rotation2d robotHeading = robotPose.getRotation();

        List<Pose2d> fuelPoses = new ArrayList<>();
        for (var pose3d : SimulatedArena.getInstance().getGamePiecesPosesByType("Fuel")) {
            fuelPoses.add(pose3d.toPose2d());
        }

        fuelPoses.sort(
                Comparator.comparingDouble(pose -> pose.getTranslation().getDistance(robotPose.getTranslation())));

        int count = Math.min(maxTargets, fuelPoses.size());
        List<TargetObservation> targets = new ArrayList<>(count);

        for (int i = 0; i < count; i++) {
            Pose2d fuel = fuelPoses.get(i);

            double dx = fuel.getX() - robotPose.getX();
            double dy = fuel.getY() - robotPose.getY();

            double rx = dx * robotHeading.getCos() + dy * robotHeading.getSin();
            double ry = -dx * robotHeading.getSin() + dy * robotHeading.getCos();

            Rotation2d yaw = new Rotation2d(rx, ry);

            targets.add(new TargetObservation(yaw, new Rotation2d()));
        }

        inputs.latestTargets = targets.toArray(new TargetObservation[0]);
    }
}
