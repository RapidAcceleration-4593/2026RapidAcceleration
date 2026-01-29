package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularMomentumUnit;
import edu.wpi.first.units.MomentOfInertiaUnit;

public class ExtraUnits {
    /** A unit of angular momentum. Note: the pound is used as a mass unit. */
    public static final AngularMomentumUnit PoundInchesSquaredPerSecond =
            Pounds.mult(InchesPerSecond).mult(Inches);
    /** A unit of moment of inertia. Note: the pound is used as a mass unit. */
    public static final MomentOfInertiaUnit PoundSquareInches = PoundInchesSquaredPerSecond.per(RadiansPerSecond);

    public static final AngularAccelerationUnit RPMPerSecond = RPM.per(Second);
}
