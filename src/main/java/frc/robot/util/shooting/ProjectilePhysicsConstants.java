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
}
