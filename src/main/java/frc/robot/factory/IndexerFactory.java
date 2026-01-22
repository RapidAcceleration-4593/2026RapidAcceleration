package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.indexer.*;

public final class IndexerFactory {

    private IndexerFactory() {}

    public static IndexerSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static IndexerSubsystem initializeReal() {
        return new IndexerSubsystem(new IndexerIOReal());
    }

    private static IndexerSubsystem initializeSim() {
        return new IndexerSubsystem(new IndexerIOSim());
    }

    private static IndexerSubsystem initializeReplay() {
        return new IndexerSubsystem(new IndexerIO() {});
    }
}
