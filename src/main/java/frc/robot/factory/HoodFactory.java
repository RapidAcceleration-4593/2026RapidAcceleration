package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.hood.*;

public final class HoodFactory {

    private HoodFactory() {}

    public static HoodSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new HoodSubsystem(new HoodIOReal());
            case SIM -> new HoodSubsystem(new HoodIOSim());
            case REPLAY -> new HoodSubsystem(new HoodIO() {});
        };
    }
}
