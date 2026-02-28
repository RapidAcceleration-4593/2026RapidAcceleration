package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    private static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    private static final double[][] kExitFactorData = {
        // { distanceMeters, turretRadians, exitFactor }
        {1.60, 0.00, 0.33},
        {2.00, 0.00, 0.35},
        {2.90, 0.00, 0.37},
        {3.60, 0.00, 0.34},
        {3.60, 0.00, 0.36},
        {4.94, 0.00, 0.29}
    };

    public static double getExitFactor(Distance distance, Angle turretAngle) {
        double d = distance.in(Meters);
        double t = turretAngle.in(Radians);

        double weightedSum = 0;
        double weightedTotal = 0;

        for (double[] p : kExitFactorData) {
            double dd = d - p[0];
            double dt = t - p[1]; // Multiply by 2.0 to increase angle weight.

            double distSquared = dd * dd + dt * dt;

            if (distSquared < 1e-6) {
                return p[2];
            }

            double weight = 1.0 / distSquared;

            weightedSum += weight * p[2];
            weightedTotal += weight;
        }
        double exitFactor = weightedSum / weightedTotal;
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

    public static Time calculateTime(LinearVelocity launchSpeed, Angle hoodAngle, Distance horizontalDistance) {
        double sin = Math.sin(hoodAngle.in(Radians));

        double numerator = horizontalDistance.in(Meters);
        double denominator = launchSpeed.in(MetersPerSecond) * sin;

        return Seconds.of(numerator / denominator);
    }
}
