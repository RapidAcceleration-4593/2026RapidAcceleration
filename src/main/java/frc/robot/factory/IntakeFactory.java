package frc.robot.factory;

import frc.robot.subsystems.intake.IntakeSubsystem;

public final class IntakeFactory {

    private IntakeFactory() {}

    public static IntakeSubsystem initialize() {
        return new IntakeSubsystem();
    }
}
