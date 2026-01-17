package frc.robot.factory;

import frc.robot.subsystems.spindexer.SpindexerSubsystem;

public final class SpindexerFactory {

    private SpindexerFactory() {}

    public static SpindexerSubsystem initialize() {
        return new SpindexerSubsystem();
    }
}
