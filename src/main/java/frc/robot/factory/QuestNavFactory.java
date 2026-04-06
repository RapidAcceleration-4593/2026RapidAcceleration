package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.vision.*;

public final class QuestNavFactory {

    private QuestNavFactory() {}

    public static QuestNavSubsystem initialize(QuestNavSubsystem.VisionConsumer swerve) {
        return switch (kCurrentMode) {
            case REAL -> new QuestNavSubsystem(new QuestNavIOReal(), swerve);
            case SIM -> new QuestNavSubsystem(new QuestNavIOReal(), swerve);
            case REPLAY -> new QuestNavSubsystem(new QuestNavIO() {}, swerve);
        };
    }
}
