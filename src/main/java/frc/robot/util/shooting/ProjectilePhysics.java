package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.shooting.ProjectilePhysicsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    public static double getExitFactor(Angle turretAngle) {
        double theta = turretAngle.in(Radians);
        double exitFactor = kA0
                + kA1 * Math.cos(theta)
                + kB1 * Math.sin(theta)
                + kA2 * Math.cos(2.0 * theta)
                + kB2 * Math.sin(2.0 * theta);

        return MathUtil.clamp(exitFactor, 0.20, 0.45);
    }

    public static LinearVelocity calculateLaunchSpeed(
            Angle hoodAngle, Distance horizontalDistance, Distance verticalDistance) {
        double sin = Math.sin(hoodAngle.in(Radians));
        double cos = Math.cos(hoodAngle.in(Radians));

        double numerator = gravity.in(MetersPerSecondPerSecond) * Math.pow(horizontalDistance.in(Meters), 2);
        double denominator = 2.0 * sin * (horizontalDistance.in(Meters) * cos - verticalDistance.in(Meters) * sin);

        return MetersPerSecond.of(Math.sqrt(numerator / denominator));
    }

    public static Time calculateTime(LinearVelocity launchSpeed, Angle hoodAngle, Distance targetHeight) {
        double y_0 = kShooterHeight.in(Meters) - targetHeight.in(Meters);
        double v_y = Math.cos(hoodAngle.in(Radians)) * launchSpeed.in(MetersPerSecond);
        double g = gravity.in(MetersPerSecondPerSecond);
        double t = (v_y + Math.sqrt(v_y * v_y + 2 * g * y_0)) / g;
        return Seconds.of(t);
    }
}
