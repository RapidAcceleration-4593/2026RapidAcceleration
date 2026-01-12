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

    public static final class TurretHardwareConstants {
        public static final SparkMax turretMotor = new SparkMax(0, MotorType.kBrushless);
        public static final Encoder turretEncoder = new Encoder(0, 0);
    }

    public static final class TurretMechanismConstants {
        public static final int driveGear = 0; // # of Teeth on smaller drive gear.
        public static final int ringGear = 0; // # of Teeth on larger ring gear.
        public static final int pulsesPerRotation = 0; // Encoder counts for one drive gear rotation.

        public static final double degreesPerPulse = (360.0 * driveGear) / (ringGear * pulsesPerRotation);

        public static final Angle initialAngle = Degrees.of(0.0);
        public static final Angle minimumAngle = Degrees.of(-160.0);
        public static final Angle maximumAngle = Degrees.of(160.0);
    }

    public static final class TurretControlConstants {
        public static final PIDController turretPID = new PIDController(0.0, 0.0, 0.0);
        public static final Angle maximumTolerance = Degrees.of(0.0);

        public static final Pose2d targetPose = new Pose2d(0.0, 0.0, new Rotation2d());
    }
}
