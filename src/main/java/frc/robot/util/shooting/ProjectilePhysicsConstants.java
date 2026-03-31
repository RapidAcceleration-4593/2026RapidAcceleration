package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.Time;

public final class ProjectilePhysicsConstants {

    public static final LinearAcceleration gravity = MetersPerSecondPerSecond.of(9.81);

    public static final Distance kShooterHeight = Inches.of(18.0);
    public static final Distance kShooterRadius = Inches.of(2.0);

    public static final int kCalculationIterations = 4;
    public static final double kConvergenceEpsilon = 3e-4;
    public static final Time kSystemLatency = Milliseconds.of(40.0);

    // Exit Factor Regression.
    public static final double kA0 = 0.375767458567097;
    public static final double kA1 = 0.01168963423034169;
    public static final double kB1 = 0.014329534629004857;
    public static final double kA2 = 0.002173455982684376;
    public static final double kB2 = -0.005075937846624539;
}
