package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {

    public static final int kSpindexerMotorID = 1;
    public static final int kFeederMotorID = 2;

    public static final Voltage kSpindexerVolts = Volts.of(6.0);
    public static final Voltage kFeederVolts = Volts.of(6.0);
}
