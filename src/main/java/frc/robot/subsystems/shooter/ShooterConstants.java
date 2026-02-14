package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class ShooterConstants {

    public static final int kMotorID = 7;

    public static final Voltage kShooterVolts = Volts.of(9.75); // Only for experimental use.

    public static final boolean kInvertMotor = false;

    // For Neo 1.0: kS = 0.39; kV = 0.00216; kA = 0.0002
    public static final double kS = 0.0;
    public static final double kV = 0.0;
    public static final double kA = 0.0;

    public static final AngularVelocity kZeroVelocity = RPM.zero();
    public static final AngularVelocity kShootVelocity = RPM.of(4200);
    public static final AngularVelocity kVelocityTolerance = RPM.of(300);

    public static final double kShooterGearing = (15.0 / 14.0);
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.0008);
}
