package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.vision.VisionConsumer;
import frc.robot.subsystems.vision.quest.QuestNavIO;
import frc.robot.subsystems.vision.quest.QuestNavIOReal;
import frc.robot.subsystems.vision.quest.QuestNavIOSim;
import frc.robot.subsystems.vision.quest.QuestNavSubsystem;
import frc.robot.util.SimulationManager;

public final class QuestNavFactory {

    private QuestNavFactory() {}

    public static QuestNavSubsystem initialize(VisionConsumer swerve) {
        return switch (kCurrentMode) {
            case REAL -> new QuestNavSubsystem(new QuestNavIOReal(), swerve);
            case SIM -> new QuestNavSubsystem(new QuestNavIOSim(SimulationManager.getInstance()::getPose), swerve);
            case REPLAY -> new QuestNavSubsystem(new QuestNavIO() {}, swerve);
        };
    }
}
