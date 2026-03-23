package frc.robot.factory;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.AprilTagConstants.*;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.AprilTagIO;
import frc.robot.subsystems.vision.AprilTagIOReal;
import frc.robot.subsystems.vision.AprilTagIOSim;
import frc.robot.subsystems.vision.AprilTagSubsystem;
import frc.robot.util.SimulationManager;
import java.util.Arrays;

public final class AprilTagFactory {

    private AprilTagFactory() {}

    public static AprilTagSubsystem initialize(SwerveSubsystem swerve) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve);
            case SIM -> initializeSim(swerve);
            case REPLAY -> initializeReplay(swerve);
        };
    }

    private static AprilTagSubsystem initializeReal(SwerveSubsystem swerve) {
        return new AprilTagSubsystem(
                swerve,
                Arrays.stream(kCameras)
                        .map(cfg -> new AprilTagIOReal(cfg.name(), cfg.robotToCamera()))
                        .toArray(AprilTagIO[]::new));
    }

    private static AprilTagSubsystem initializeSim(SwerveSubsystem swerve) {
        SimulationManager simulation = SimulationManager.getInstance();
        return new AprilTagSubsystem(
                swerve,
                Arrays.stream(kCameras)
                        .map(cfg -> new AprilTagIOSim(cfg.name(), cfg.robotToCamera(), simulation::getPose))
                        .toArray(AprilTagIO[]::new));
    }

    private static AprilTagSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new AprilTagSubsystem(
                swerve, Arrays.stream(kCameras).map(cfg -> new AprilTagIO() {}).toArray(AprilTagIO[]::new));
    }
}
