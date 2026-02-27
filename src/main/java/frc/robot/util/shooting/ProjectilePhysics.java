package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    private static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

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
