package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.hood.HoodConstants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.kPhysicalOffset;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class HoodSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final HoodInputsAutoLogged inputs;
    private final HoodIO io;

    private final LoggedMechanism2d mechanism;
    private final LoggedMechanismRoot2d root;
    private final LoggedMechanismLigament2d hood;

    public HoodSubsystem(HoodIO io, Supplier<Pose2d> poseSupplier) {
        this.io = io;
        this.inputs = new HoodInputsAutoLogged();
        this.poseSupplier = poseSupplier;

        mechanism = new LoggedMechanism2d(1.0, 1.0);
        root = mechanism.getRoot("HoodRoot", 0.5, 0.5);
        hood = root.append(new LoggedMechanismLigament2d("Hood", Inches.of(12), kMinimumAngle));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        io.updateControl();
        Logger.processInputs("Hood", inputs);

        hood.setAngle(inputs.angle);
        Logger.recordOutput("Mechanisms/Hood", mechanism);
    }

    public void setAngle(Angle angle) {
        io.setAngle(angle);
    }

    public Angle getAngle() {
        return inputs.angle;
    }

    public boolean atAngle() {
        return inputs.atTargetAngle;
    }

    public void stop() {
        io.stop();
    }

    /** Sets the angle based on the robot's current pose relative to the Hub. */
    public void setAngleToHub() {
        Pose2d targetPose = kAlliance == Alliance.Blue ? kBlueHubPose : kRedHubPose;
        Pose2d shooterPose = poseSupplier.get().plus(kPhysicalOffset);

        Distance distance = Meters.of(shooterPose.getTranslation().getDistance(targetPose.getTranslation()));
        Angle targetAngle = Degrees.of(5 * distance.in(Meters));

        setAngle(targetAngle);
    }
}
