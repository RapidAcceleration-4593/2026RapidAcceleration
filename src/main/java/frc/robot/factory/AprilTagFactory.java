package frc.robot.factory;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.vision.apriltag.AprilTagConstants.*;

import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.vision.apriltag.*;
import frc.robot.util.Simulation;
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
                        .map(cfg -> new AprilTagIOPhotonVision(cfg.name(), cfg.robotToCamera()))
                        .toArray(AprilTagIO[]::new));
    }

    private static AprilTagSubsystem initializeSim(SwerveSubsystem swerve) {
        Simulation simulation = Simulation.getInstance();
        return new AprilTagSubsystem(
                swerve,
                Arrays.stream(kCameras)
                        .map(cfg -> new AprilTagIOPhotonVisionSim(cfg.name(), cfg.robotToCamera(), simulation::getPose))
                        .toArray(AprilTagIO[]::new));
    }

    private static AprilTagSubsystem initializeReplay(SwerveSubsystem swerve) {
        return new AprilTagSubsystem(
                swerve, Arrays.stream(kCameras).map(cfg -> new AprilTagIO() {}).toArray(AprilTagIO[]::new));
    }
}
