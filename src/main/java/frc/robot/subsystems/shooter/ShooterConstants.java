package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {

    public static final int kShooterMotorID = 0;

    public static final double kShooterGearRatio = 1.0;
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.002);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final AngularVelocity kDefaultShooterRPM = RPM.of(2000.0);
    public static final AngularVelocity kVelocityToleranceRPM = RPM.of(50.0);

    public static final Current kSmartCurrentLimit = Amps.of(40);
}
