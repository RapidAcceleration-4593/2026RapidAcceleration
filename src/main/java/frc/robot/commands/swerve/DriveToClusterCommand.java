package frc.robot.commands.swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.objectdetection.ObjectDetectionSubsystem;
import java.util.Set;

public class DriveToClusterCommand extends DeferredCommand {

    private static final double kRotationGain = 3.0;
    private static final double kMaxAngularVelocity = 4.0;
    private static final double kForwardVelocity = 2.0;

    public DriveToClusterCommand(SwerveSubsystem swerve, ObjectDetectionSubsystem objectdetection) {
        super(
                () -> swerve.run(() -> {
                    double vx = kForwardVelocity;
                    double vy = 0.0;

                    Rotation2d clusterYaw = objectdetection.getClusterYaw();
                    double yawRad = MathUtil.clamp(clusterYaw.getRadians(), -Math.PI / 4, Math.PI / 4);
                    double omega = MathUtil.clamp(yawRad * kRotationGain, -kMaxAngularVelocity, kMaxAngularVelocity);

                    ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, omega);
                    swerve.runVelocity(speeds);
                }),
                Set.of(swerve, objectdetection));
    }
}
