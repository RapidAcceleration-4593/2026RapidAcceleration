package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;
import static frc.robot.util.shooting.ProjectilePhysicsConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    public static double getExitFactor(Distance distance, Angle turretAngle) {
        double x = distance.in(Meters);
        double y = turretAngle.in(Radians);

        for (int[] tri : kTriangles) {
            double[] p1 = kExitFactorData[tri[0]];
            double[] p2 = kExitFactorData[tri[1]];
            double[] p3 = kExitFactorData[tri[2]];

            double x1 = p1[0], y1 = p1[1], z1 = p1[2];
            double x2 = p2[0], y2 = p2[1], z2 = p2[2];
            double x3 = p3[0], y3 = p3[1], z3 = p3[2];

            double denom = (y2 - y3) * (x1 - x3) + (x3 - x2) * (y1 - y3);

            if (Math.abs(denom) < 1e-9) continue;

            double w1 = ((y2 - y3) * (x - x3) + (x3 - x2) * (y - y3)) / denom;
            double w2 = ((y3 - y1) * (x - x3) + (x1 - x3) * (y - y3)) / denom;
            double w3 = 1.0 - w1 - w2;

            if (w1 >= 0 && w2 >= 0 && w3 >= 0) {
                double exitFactor = w1 * z1 + w2 * z2 + w3 * z3;
                return MathUtil.clamp(exitFactor, 0.20, 0.45);
            }
        }

        return kExitFactorData[0][2];
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
