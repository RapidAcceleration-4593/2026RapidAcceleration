package frc.robot.factory;

import static frc.robot.Constants.kCurrentMode;

import frc.robot.subsystems.turret.*;

public final class TurretFactory {

    private TurretFactory() {}

    public static TurretSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new TurretSubsystem(new TurretIOReal());
            case SIM -> new TurretSubsystem(new TurretIOSim());
            case REPLAY -> new TurretSubsystem(new TurretIO() {});
        };
    }
}
