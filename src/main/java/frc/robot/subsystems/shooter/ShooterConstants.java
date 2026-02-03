package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {

    public static final int kShooterMotorID = 1;

    public static final double kP = 0.0011;
    public static final double kI = 0.001;
    public static final double kD = 0.0;

    public static final double kS = 0.0; // Voltage to overcome static friction.
    public static final double kV = 0.0; // Volts per RPM to maintain speed.
    public static final double kA = 0.0; // Optional for acceleration.

    public static final AngularVelocity kCruiseVelocity = RPM.of(3000);
    public static final AngularAcceleration kMaxAcceleration = RPM.per(Second).of(4000);
    public static final AngularVelocity kZeroVelocity = RPM.zero();
    public static final AngularVelocity kShootVelocity = RPM.of(3000.0);
    public static final AngularVelocity kVelocityTolerance = RPM.of(50.0);

    public static final double kShooterGearing = 1.0;
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.000434);
}
