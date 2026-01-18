package frc.robot.subsystems.hood;

import static frc.robot.Constants.*;
import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class HoodSubsystem extends SubsystemBase {

    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    private final Supplier<Pose2d> poseSupplier;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();

        this.poseSupplier = poseSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateControl();
        Logger.processInputs("Hood", inputs);
    }

    public void setAngle(Angle angle) {
        io.setAngle(angle);
    }

    public Angle getAngle() {
        return io.getAngle();
    }

    public boolean atAngle() {
        return io.atAngle();
    }

    public void stop() {
        io.stop();
    }

    public void setAngleFromDistance() {
        Distance distance = calculateDistance(poseSupplier.get());
        Angle angle = calculateAngle(distance);
        setAngle(angle);
    }

    private Distance calculateDistance(Pose2d pose) {
        Pose2d targetPose = kAlliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
        Pose2d shooterPose = pose.plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));
        return distance;
    }

    private Angle calculateAngle(Distance distance) {
        return Degrees.of(5 * distance.in(Meters));
    }
}
