package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.kPhysicalOffset;
import static frc.robot.subsystems.turret.TurretConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommandLogger;
import frc.robot.util.FieldUtil;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class TurretSubsystem extends SubsystemBase {

    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;
    private final TurretInputsAutoLogged inputs;
    private final TurretIO io;

    private Angle targetAngle = kInitialAngle;

    public TurretSubsystem(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
        this.io = io;
        this.inputs = new TurretInputsAutoLogged();
        this.poseSupplier = poseSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Turret", inputs);
        targetAngle = inputs.targetAngle;

        CommandLogger.logSubsystemCommand(this);
    }

    public Angle getCurrentAngle() {
        return inputs.angle;
    }

    public Angle getTargetAngle() {
        return inputs.targetAngle;
    }

    public boolean atTargetAngle() {
        return inputs.angle.isNear(targetAngle, kAngleTolerance);
    }

    /**
     * Constructs a command to run the turret at a set voltage.
     *
     * @param volts The voltage to apply to the motor.
     * @return A command to set the motor voltage and stop when complete.
     */
    public Command setVoltageCommand(Voltage volts) {
        return startEnd(() -> io.setVoltage(volts), io::stop)
                .until(() -> (inputs.angle.lt(kMinimumAngle) || inputs.angle.gt(kMaximumAngle)));
    }

    /**
     * Constructs a command to run the turret to a set angle.
     *
     * @param angle The angle to apply to the closed-loop PID control.
     * @return A command to run the motor to an angle and stop when complete.
     */
    public Command goToAngleCommand(Angle angle) {
        return startEnd(() -> setPosition(angle), io::stop).until(this::atTargetAngle);
    }

    /**
     * Constructs a command to continuously run the turret to the calculated Hub angle.
     *
     * @return A command to run the motor to the calculated Hub angle without stopping.
     */
    public Command controlAngleCommand() {
        return runEnd(() -> setPosition(calculateSafeAngle()), io::stop);
    }

    /**
     * Constructs a command to stop the turret motor.
     *
     * @return A command to stop the motor immediately.
     */
    public Command stopCommand() {
        return runOnce(io::stop);
    }

    /** Sets the angle of the closed-loop PID control. */
    private void setPosition(Angle angle) {
        Angle clampedAngle =
                Degrees.of(MathUtil.clamp(angle.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        targetAngle = clampedAngle;
        io.setPosition(clampedAngle);
    }

    /**
     * Calculates the angle based on the robot's rotation from the hub.
     *
     * @return An angle supplier from a linear regression equation.
     */
    private Angle calculateTurretAngle() {
        Pose2d robotPose = poseSupplier.get().transformBy(kPhysicalOffset);

        if (FieldUtil.isInAllianceZone(robotPose)) {
            Pose2d targetPose = FieldUtil.getTargetHubPose();
            ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();

            Distance vx = Meters.of(chassisSpeeds.vxMetersPerSecond);
            Distance vy = Meters.of(chassisSpeeds.vyMetersPerSecond);

            Distance dx = targetPose.getMeasureX().minus(vx).minus(robotPose.getMeasureX());
            Distance dy = targetPose.getMeasureY().minus(vy).minus(robotPose.getMeasureY());

            Angle fieldAngle = Radians.of(Math.atan2(dy.in(Meters), dx.in(Meters)));
            return robotPose.getRotation().getMeasure().minus(fieldAngle);
        }

        Angle fieldAngle = FieldUtil.getCurrentAlliance() == Alliance.Blue ? Degrees.of(180) : Degrees.zero();
        return robotPose.getRotation().getMeasure().minus(fieldAngle);
    }

    /**
     * Calculates a safe angle to run the turret to based on the current and desired angles.
     *
     * @return A safe angle that does not exceed the physical limits of the turret.
     */
    private Angle calculateSafeAngle() {
        Angle current = getCurrentAngle();
        Angle desired = calculateTurretAngle();

        Angle error = Degrees.of(MathUtil.inputModulus(desired.minus(current).in(Degrees), -180.0, 180.0));

        Angle candidate = current.plus(error);

        if (candidate.lt(kMinimumAngle) || candidate.gt(kMaximumAngle)) {
            return Degrees.of(
                    MathUtil.clamp(desired.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
        }

        return Degrees.of(MathUtil.clamp(candidate.in(Degrees), kMinimumAngle.in(Degrees), kMaximumAngle.in(Degrees)));
    }
}
