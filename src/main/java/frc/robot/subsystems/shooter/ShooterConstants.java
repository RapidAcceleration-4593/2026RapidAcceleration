package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class ShooterConstants {

    public static final int kShooterMotorID = 7;

    public static final Voltage kShooterVolts = Volts.of(9.75); // Only for experimental use.

    public static final double kP = 0.0; // 0.0011
    public static final double kI = 0.0; // 0.001
    public static final double kD = 0.0;

    public static final double kS = 0.39; // Voltage to overcome static friction.
    public static final double kV = 0.002267; // Volts per RPM to maintain speed.

    public static final AngularVelocity kZeroVelocity = RPM.zero();
    public static final AngularVelocity kShootVelocity = RPM.of(4200);
    public static final AngularVelocity kVelocityTolerance = RPM.of(50);
    public static final AngularVelocity kCruiseVelocity = RPM.of(4200);
    public static final AngularAcceleration kMaxAcceleration = RPM.per(Second).of(3000);

    public static final double kShooterGearing = 1.0;
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.000434);
}
