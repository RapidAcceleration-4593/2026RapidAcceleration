package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class ProjectilePhysics {

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);
    public static final Distance kShooterHeight = Inches.of(18.0);
    public static final Distance kWheelRadius = Inches.of(2.0);

    public static LinearVelocity calculateLaunchSpeed(
            Angle hoodAngle, Distance horizontalDistance, Distance verticalDistance) {
        double sin = Math.sin(hoodAngle.in(Radians));
        double cos = Math.cos(hoodAngle.in(Radians));

        double numerator = gravity.in(MetersPerSecondPerSecond) * Math.pow(horizontalDistance.in(Meters), 2);
        double denominator = 2.0 * sin * (horizontalDistance.in(Meters) * cos + verticalDistance.in(Meters) * sin);

        return MetersPerSecond.of(Math.sqrt(numerator / denominator));
    }

    public static Time calculateTime(LinearVelocity launchSpeed, Angle hoodAngle, Distance verticalDistance) {
        double y_0 = verticalDistance.in(Meters);
        double v_y = Math.cos(hoodAngle.in(Radians)) * launchSpeed.in(MetersPerSecond);
        double g = gravity.in(MetersPerSecondPerSecond);
        double t = (v_y + Math.sqrt(v_y * v_y + 2 * g * y_0)) / g;
        return Seconds.of(t);
    }
}
