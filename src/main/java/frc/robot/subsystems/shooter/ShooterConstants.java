package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class ShooterConstants {

    public static final int kShooterMotorID = 7;

    public static final Voltage kShooterVolts = Volts.of(9.75); // Only for experimental use.

    public static final double kP = 0.0; // 0.00005
    public static final double kI = 0.0; // 0.0000003
    public static final double kD = 0.0;

    public static final double kS = 0.39; // 0.39
    public static final double kV = 0.00216;
    public static final double kA = 0.0002;

    public static final AngularVelocity kZeroVelocity = RPM.zero();
    public static final AngularVelocity kShootVelocity = RPM.of(4100);
    public static final AngularVelocity kVelocityTolerance = RPM.of(300);
    public static final AngularAcceleration kMaxAcceleration = RPM.per(Second).of(10000);

    public static final double kShooterGearing = 1.0; // V2: 15.0 / 15.0, might change to 15.0 / 14.0
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.0008);
}
