package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {

    public static final int kSpindexerMotorID = 3;
    public static final int kFeederMotorID = 4;

    public static final Voltage kSpindexerVolts = Volts.of(0.5);
    public static final Voltage kFeederVolts = Volts.of(3.0);
}
