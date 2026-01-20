package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.ExtraUnits.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {

    public static final int kShooterMotorID = 3;

    public static final double kP = 0.0; // 01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0; // Voltage to overcome static friction.
    public static final double kV = 0.0017; // Volts per RPM to maintain speed.
    public static final double kA = 0.0; // Optional for acceleration.

    public static final AngularAcceleration kMaxAcceleration = RPMPerSecond.of(500);

    public static final AngularVelocity kDefaultShooterRPM = RPM.of(1200.0);
    public static final AngularVelocity kVelocityTolerance = RPM.of(50.0);

    public static final Transform2d kPhysicalOffset =
            new Transform2d(new Translation2d(Inches.of(-5.375), Inches.zero()), new Rotation2d());

    public static final MomentOfInertia kShooterWheelMOI = PoundSquareInches.of(1.5);
    public static final MomentOfInertia kHoodWheelMOI =
            PoundSquareInches.of(0.14); // TODO: add hood wheel to simulation
    public static final double kShooterWheelGearing = 1;
    public static final double kHoodWheelGearing = 1;
}
