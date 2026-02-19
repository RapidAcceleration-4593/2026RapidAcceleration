package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {

    public static final int kMotorID = 7;

    public static final boolean kInvertMotor = false;

    public static final Slot0Configs kShooterGains = new Slot0Configs()
            .withKS(0.1) // Voltage to overcome static friction.
            .withKV(0.11) // Voltage per RPS to maintain the target velocity.
            .withKA(0.0) // Voltage per RPS/s to accelerate the shooter.
            .withKP(0.0) // Error correction, often minimal.
            .withKI(0.0) // Often not necessary for shooters.
            .withKD(0.0) // Use sparingly to dampen overshoots.
            .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    public static final AngularVelocity kZeroVelocity = RPM.zero();
    public static final AngularVelocity kShootVelocity = RPM.of(4000);
    public static final AngularVelocity kVelocityTolerance = RPM.of(300);

    public static final double kShooterGearing = (15.0 / 14.0);
    public static final MomentOfInertia kShooterMOI = KilogramSquareMeters.of(0.0008);
}
