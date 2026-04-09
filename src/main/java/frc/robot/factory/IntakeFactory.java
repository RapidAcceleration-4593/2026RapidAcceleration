package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.intake.*;

public final class IntakeFactory {

    private IntakeFactory() {}

    public static IntakeSubsystem initialize() {
        return switch (kCurrentMode) {
            case REAL -> new IntakeSubsystem(new IntakeIOReal());
            case SIM -> new IntakeSubsystem(new IntakeIOSim());
            case REPLAY -> new IntakeSubsystem(new IntakeIO() {});
        };
    }
}
