package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

public class ObjectDetectionSubsystem extends SubsystemBase {

    private final ObjectDetectionInputsAutoLogged inputs;
    private final Supplier<Pose2d> robotPoseSupplier;
    private final ObjectDetectionIO io;

    public ObjectDetectionSubsystem(ObjectDetectionIO io, Supplier<Pose2d> robotPoseSupplier) {
        this.io = io;
        this.robotPoseSupplier = robotPoseSupplier;
        this.inputs = new ObjectDetectionInputsAutoLogged();
    }

    public Pose2d getObjectPose() {
        if (inputs.detectedPoses.length == 0) {
            return robotPoseSupplier.get();
        }

        Pose2d density = inputs.detectedPoses[0];
        return new Pose2d(density.getX(), density.getY(), density.getRotation());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
