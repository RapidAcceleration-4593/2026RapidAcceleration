package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MomentOfInertia;

public class SpindexerConstants {

    public static final int kSpindexerMotorID = 0;
    public static final int kFeederMotorID = 0;

    public static final double kSpindexerSpeed = 0.3;
    public static final double kFeederSpeed = 0.5;

    public static final double kGearRatio = 1.0;
    public static final MomentOfInertia kMOI = KilogramSquareMeters.of(0.0002);
}
