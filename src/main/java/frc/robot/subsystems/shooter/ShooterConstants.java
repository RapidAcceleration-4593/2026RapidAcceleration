package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterConstants {

    public static final int kShooterMotorID = 0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double kS = 0.0; // Voltage to overcome static friction.
    public static final double kV = 0.0; // Volts per RPM to maintain speed.
    public static final double kA = 0.0; // Optional for acceleration.

    public static final AngularVelocity kDefaultShooterRPM = RPM.of(1200.0);
    public static final AngularVelocity kVelocityTolerance = RPM.of(50.0);

    public static final Translation2d kPhysicalOffset = new Translation2d(-Units.inchesToMeters(5.375), 0);
}
