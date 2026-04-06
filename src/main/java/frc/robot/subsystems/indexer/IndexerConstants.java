package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {

    public static final int kSpindexerMotorID = 1;
    public static final int kFeederMotorID = 3;

    public static final int kSensorChannel = 2;

    public static final double kSpindexerP = 0.0025;
    public static final double kSpindexerI = 0.00;
    public static final double kSpindexerD = 0.00;

    public static final double kFeederP = 0.0025;
    public static final double kFeederI = 0.00;
    public static final double kFeederD = 0.00;

    public static final boolean kInvertSpindexerMotor = false;
    public static final boolean kInvertFeederMotor = false;
    public static final boolean kInvertSensor = false;

    public static final Voltage kSpindexerVolts = Volts.of(12.0);
    public static final Voltage kFeederVolts = Volts.of(12.0);

    public static final AngularVelocity kSpindexerVelocity = RPM.of(4000);
    public static final AngularVelocity kFeederVelocity = RPM.of(4000);

    public static final double kSpindexerGearing = (5.0 * 3.0 * 3.0);
    public static final double kFeederGearing = (3.0);

    public static final MomentOfInertia kSpindexerMOI = KilogramSquareMeters.of(0.001);
    public static final MomentOfInertia kFeederMOI = KilogramSquareMeters.of(0.001);
}
