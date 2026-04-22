package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {

    public static final int kMotorID = 14;

    public static final boolean kInvertMotor = false;

    public static final double kS = 0.2;
    public static final double kV = 0.1175;
    public static final double kA = 0.05;

    public static final AngularVelocity kVelocityTolerance = RPM.of(300);

    public static final double kShooterGearing = (15.0 / 14.0);
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.0008);

    public static final Distance kShooterHeight = Inches.of(18.0);
    public static final Transform2d kPhysicalOffset =
            new Transform2d(new Translation2d(Inches.of(-5.557), Inches.zero()), new Rotation2d());
}
