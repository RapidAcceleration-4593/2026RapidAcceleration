package frc.robot.subsystems.vision.objectdetection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ObjectDetectionSubsystem extends SubsystemBase {

    private final ObjectDetectionInputsAutoLogged inputs;
    private final ObjectDetectionIO io;

    public ObjectDetectionSubsystem(ObjectDetectionIO io) {
        this.io = io;
        this.inputs = new ObjectDetectionInputsAutoLogged();
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Rotation2d getClusterYaw() {
        if (inputs.latestTargets.length == 0) return new Rotation2d();

        double sumX = 0;
        double sumY = 0;

        for (var target : inputs.latestTargets) {
            sumX += Math.cos(target.yaw().getRadians());
            sumY += Math.sin(target.yaw().getRadians());
        }

        return new Rotation2d(sumX, sumY);
    }
}
