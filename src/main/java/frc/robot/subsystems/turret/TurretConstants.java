package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Encoder;

public final class TurretConstants {

    // Hardware Constants.
    public static final SparkMax kTurretMotor = new SparkMax(0, MotorType.kBrushless);
    public static final Encoder kTurretEncoder = new Encoder(0, 0);

    // Mechanism Constants.
    public static final int kDriveGear = 0; // # of Teeth on smaller drive gear.
    public static final int kRingGear = 0; // # of Teeth on larger ring gear.
    public static final int kPulsesPerRotation = 0; // Encoder counts for one drive gear rotation.

    public static final double kDegreesPerPulse = (360.0 * kDriveGear) / (kRingGear * kPulsesPerRotation);

    public static final Angle kInitialAngle = Degrees.of(0.0);
    public static final Angle kMinimumAngle = Degrees.of(-160.0);
    public static final Angle kMaximumAngle = Degrees.of(160.0);

    // Control Constants.
    public static final PIDController kTurretPID = new PIDController(0.0, 0.0, 0.0);
    public static final Angle kMaximumTolerance = Degrees.of(0.0);

    public static final Pose2d kTargetPose = new Pose2d(0.0, 0.0, new Rotation2d());
}
