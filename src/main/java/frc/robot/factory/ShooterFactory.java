package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.shooter.*;

public final class ShooterFactory {

    private ShooterFactory() {}

    public static ShooterSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new ShooterSubsystem(new ShooterIOReal());
            case SIM -> new ShooterSubsystem(new ShooterIOSim());
            case REPLAY -> new ShooterSubsystem(new ShooterIO() {});
        };
    }
}
