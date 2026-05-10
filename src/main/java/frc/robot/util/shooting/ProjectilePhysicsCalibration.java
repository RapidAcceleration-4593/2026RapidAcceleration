package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;

public class ProjectilePhysicsCalibration {

    // Launch Speed [m/s], Turret Angle [rad], Exit Factor.
    public static final double[][] kExitFactorData = {
        {7.2157, -0.0358, 0.400},
        {7.6178, -0.3159, 0.390},
        {8.0006, 0.70086, 0.395},
        {8.4218, -0.0767, 0.390},
        {7.8484, -1.5037, 0.350},
        {8.0548, 1.21500, 0.390},
        {8.7895, -0.5796, 0.370},
        {8.1888, 2.14742, 0.390},
        {8.0304, -2.3390, 0.360},
        {8.8969, 2.31104, 0.370},
        {8.8849, -0.7583, 0.355},
        {8.9060, 0.65343, 0.360},
        {8.8257, -2.3365, 0.330},
        {8.1637, 1.57358, 0.395},
        {8.0803, -1.4428, 0.360},
        {8.0456, 1.37994, 0.400},
        {8.1221, -1.7379, 0.370},
        {6.4862, 2.80408, 0.370},
        {6.5577, -0.0598, 0.400},
        {6.7053, -0.7293, 0.400},
        {8.1713, 2.93616, 0.375},
        {8.7000, 1.74501, 0.390},
        {8.0536, -2.6555, 0.360},
        {8.2419, 2.85171, 0.375},
        {8.3141, -2.4599, 0.355},
        {7.9750, -1.5851, 0.360},
        {8.2545, -2.5723, 0.355},
        {8.1983, 2.84265, 0.380},
        {7.8949, -2.5504, 0.355},
        {8.5502, 1.62621, 0.390},
        {8.5458, 1.89361, 0.375},
        {7.9235, -1.0960, 0.365},
        {7.0622, -1.1685, 0.385},
        {9.0211, 2.37619, 0.360},
        {8.1086, -1.3939, 0.360},
        {7.2104, -0.0595, 0.390},
        {8.3016, -2.2050, 0.35},
        {7.9962, -1.8766, 0.345},
        {8.1041, -1.2753, 0.365},
        {8.0736, 1.75879, 0.365},
        {8.0611, 1.13120, 0.365},
        {8.4669, 1.22317, 0.3675},
        {8.6246, 3.08061, 0.360},
        {8.8230, -0.6208, 0.350},
        {8.5174, 1.04572, 0.370},
        {6.9971, -3.0165, 0.380},
        {6.8192, -1.5612, 0.380}
    };

    public static double getHarmonicExitFactor(LinearVelocity launchSpeed, Angle turretAngle) {
        double x = launchSpeed.in(MetersPerSecond);
        double y = turretAngle.in(Radians);

        double cosA = Math.cos(y);
        double sinA = Math.sin(y);
        double cos2A = Math.cos(2 * y);
        double sin2A = Math.sin(2 * y);

        double exitFactor = -0.081824
                + (0.130601 * x)
                + (-0.009162 * x * x)
                + (0.028281 * cosA)
                + (-0.055456 * sinA)
                + (-0.002500 * x * cosA)
                + (0.008522 * x * sinA)
                + (0.004287 * cos2A)
                + (-0.005829 * sin2A);

        return MathUtil.clamp(exitFactor, 0.15, 0.50);
    }
}
