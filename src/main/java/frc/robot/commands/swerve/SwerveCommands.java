package frc.robot.commands.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.util.FieldUtil;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class SwerveCommands {

    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Seconds.
    private static final double FF_RAMP_RATE = 0.1; // Volts per Second.
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Radians per Second.
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Radians per Squared Second.

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply Deadband and square for finer control.
        double magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        magnitude *= magnitude;

        double angle = Math.atan2(y, x);
        return new Translation2d(magnitude * Math.cos(angle), magnitude * Math.sin(angle));
    }

    private static ChassisSpeeds fieldRelativeSpeeds(SwerveSubsystem swerve, Translation2d linear, double omega) {
        boolean flipped = FieldUtil.isRedAlliance();
        Rotation2d robotRotation = flipped ? swerve.getRotation().plus(new Rotation2d(Math.PI)) : swerve.getRotation();
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                linear.getX() * swerve.getMaxLinearSpeedMetersPerSec(),
                linear.getY() * swerve.getMaxLinearSpeedMetersPerSec(),
                omega,
                robotRotation);
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            SwerveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity.
                    Translation2d linear =
                            getLinearVelocityFromJoysticks(-xSupplier.getAsDouble(), -ySupplier.getAsDouble());

                    // Apply rotation deadband. Square rotation value for more precise control.
                    double omega = MathUtil.applyDeadband(-omegaSupplier.getAsDouble(), DEADBAND);
                    omega = Math.copySign(omega * omega, omega);
                    omega *= swerve.getMaxAngularSpeedRadPerSec();

                    swerve.runVelocity(fieldRelativeSpeeds(swerve, linear, omega));
                },
                swerve);
    }

    /**
     * Drive field-oriented with left joystick controlling translation, but the robot automatically rotates to face a
     * fixed target rotation.
     */
    public static Command joystickDrivePointToHub(
            SwerveSubsystem swerve, DoubleSupplier xSupplier, DoubleSupplier ySupplier) {
        ProfiledPIDController angleController = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        return Commands.defer(
                () -> {
                    Pose2d targetPose = FieldUtil.getTargetHubPose();
                    angleController.reset(swerve.getRotation().getRadians());

                    return Commands.run(
                            () -> {
                                Translation2d linear = getLinearVelocityFromJoysticks(
                                        -xSupplier.getAsDouble(), -ySupplier.getAsDouble());
                                Rotation2d toTarget = targetPose
                                        .getTranslation()
                                        .minus(swerve.getPose().getTranslation())
                                        .getAngle();

                                double omega = angleController.calculate(
                                        swerve.getRotation().getRadians(), toTarget.getRadians());
                                swerve.runVelocity(fieldRelativeSpeeds(swerve, linear, omega));
                            },
                            swerve);
                },
                Set.of(swerve));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(SwerveSubsystem swerve) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data.
                Commands.runOnce(() -> {
                    velocitySamples.clear();
                    voltageSamples.clear();
                }),

                // Allow modules to orient.
                Commands.run(() -> swerve.runCharacterization(0.0), swerve).withTimeout(FF_START_DELAY),

                // Start timer.
                Commands.runOnce(timer::restart),

                // Accelerate and gather data.
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    swerve.runCharacterization(voltage);
                                    velocitySamples.add(swerve.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                swerve)
                        .finallyDo(() -> {
                            int n = velocitySamples.size();
                            double sumX = 0.0;
                            double sumY = 0.0;
                            double sumXY = 0.0;
                            double sumX2 = 0.0;
                            for (int i = 0; i < n; i++) {
                                double v = velocitySamples.get(i);
                                double u = voltageSamples.get(i);
                                sumX += v;
                                sumY += u;
                                sumXY += v * u;
                                sumX2 += v * v;
                            }
                            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                            NumberFormat formatter = new DecimalFormat("#0.00000");
                            System.out.println("********** Drive FF Characterization Results **********");
                            System.out.println("\tkS: " + formatter.format(kS));
                            System.out.println("\tkV: " + formatter.format(kV));
                        }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(SwerveSubsystem swerve) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence.
                Commands.sequence(
                        // Reset acceleration limiter.
                        Commands.runOnce(() -> limiter.reset(0.0)),

                        // Turn in place, accelerating up to full speed.
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    swerve.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                swerve)),

                // Measurement sequence.
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement.
                        Commands.waitSeconds(1.0),

                        // Record starting measurement.
                        Commands.runOnce(() -> {
                            state.positions = swerve.getWheelRadiusCharacterizationPositions();
                            state.lastAngle = swerve.getRotation();
                            state.gyroDelta = 0.0;
                        }),

                        // Update gyro delta.
                        Commands.run(() -> {
                                    Rotation2d rotation = swerve.getRotation();
                                    state.gyroDelta += Math.abs(
                                            rotation.minus(state.lastAngle).getRadians());
                                    state.lastAngle = rotation;
                                })
                                .finallyDo(() -> {
                                    double[] positions = swerve.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * kDriveBaseRadius) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println("\tWheel Radius: " + formatter.format(wheelRadius) + " meters, "
                                            + formatter.format(Units.metersToInches(wheelRadius)) + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }
}
