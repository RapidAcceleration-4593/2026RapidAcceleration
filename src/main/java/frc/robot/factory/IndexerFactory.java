package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.indexer.*;

public final class IndexerFactory {

    private IndexerFactory() {}

    public static IndexerSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new IndexerSubsystem(new IndexerIOReal());
            case SIM -> new IndexerSubsystem(new IndexerIOSim());
            case REPLAY -> new IndexerSubsystem(new IndexerIO() {});
        };
    }
}
