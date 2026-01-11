package frc.robot.subsystems.vision.objectdetection;

import static frc.robot.subsystems.vision.objectdetection.ObjectDetectionConstants.cameraTransform;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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

        Pose2d objectPose;
        if (io instanceof ObjectDetectionIOSim) {
            objectPose = inputs.detectedPoses[0];
        } else {
            Pose2d cameraRelative = inputs.detectedPoses[0];
            Pose2d robotPose = robotPoseSupplier.get();
            objectPose = robotPose
                    .plus(cameraTransform)
                    .plus(new Transform2d(cameraRelative.getTranslation(), cameraRelative.getRotation()));
        }

        Pose2d robotPose = robotPoseSupplier.get();
        Translation2d direction = objectPose.getTranslation().minus(robotPose.getTranslation());

        Rotation2d rotationToObject = new Rotation2d(Math.atan2(direction.getY(), direction.getX()));
        return new Pose2d(objectPose.getTranslation(), rotationToObject);
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }
}
