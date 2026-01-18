package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.spindexer.*;

public final class SpindexerFactory {

    private SpindexerFactory() {}

    public static SpindexerSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static SpindexerSubsystem initializeReal() {
        return new SpindexerSubsystem(new SpindexerIOReal());
    }

    private static SpindexerSubsystem initializeSim() {
        return new SpindexerSubsystem(new SpindexerIOSim());
    }

    private static SpindexerSubsystem initializeReplay() {
        return new SpindexerSubsystem(new SpindexerIO() {});
    }
}
