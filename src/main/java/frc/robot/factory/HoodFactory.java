package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.hood.*;

public final class HoodFactory {

    private HoodFactory() {}

    public static HoodSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> initializeReal();
            case SIM -> initializeSim();
            case REPLAY -> initializeReplay();
        };
    }

    private static HoodSubsystem initializeReal() {
        return new HoodSubsystem(new HoodIOReal());
    }

    private static HoodSubsystem initializeSim() {
        return new HoodSubsystem(new HoodIOSim());
    }

    private static HoodSubsystem initializeReplay() {
        return new HoodSubsystem(new HoodIO() {});
    }
}
