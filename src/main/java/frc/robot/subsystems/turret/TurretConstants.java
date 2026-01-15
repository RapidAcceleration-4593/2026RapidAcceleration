package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Encoder;

public final class TurretConstants {

    // Hardware Constants.
    public static final SparkMax kTurretMotor = new SparkMax(0, MotorType.kBrushless);
    public static final Encoder kTurretEncoder = new Encoder(0, 1);

    // Mechanism Constants.
    public static final double kGearboxRatio = 1; // Gear ratio between motor and drive gear.
    public static final int kDriveGear = 12; // # of Teeth on smaller drive gear.
    public static final int kRingGear = 80; // # of Teeth on larger ring gear.
    public static final double kMotorTurretGearing =
            kGearboxRatio * kRingGear / kDriveGear; // Gear ratio between motor and turret.

    public static final MomentOfInertia kTurretMOI = KilogramSquareMeters.of(0.0030);
    public static final int kPulsesPerRotation = 8192; // Encoder counts for one drive gear rotation.

    public static final double kMotorRotationsPerPulse = 1.0 / kPulsesPerRotation;
    public static final double kTurretRotationsPerPulse = kDriveGear / kRingGear;
    public static final double kDegreesPerPulse = 360 * kMotorRotationsPerPulse * kTurretRotationsPerPulse;

    public static final Angle kInitialAngle = Degrees.of(0.0);
    public static final Angle kMinimumAngle = Degrees.of(-160.0);
    public static final Angle kMaximumAngle = Degrees.of(160.0);

    // Control Constants.
    public static final double kTurretP = 0.0;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.0;

    public static final Angle kMaximumTolerance = Degrees.of(0.0);

    public static final Pose2d kBlueHubPose =
            new Pose2d(Units.inchesToMeters(182.1), Units.inchesToMeters(158.85), new Rotation2d());
    public static final Pose2d kRedHubPose =
            new Pose2d(Units.inchesToMeters(469.1), Units.inchesToMeters(158.85), new Rotation2d());
}
