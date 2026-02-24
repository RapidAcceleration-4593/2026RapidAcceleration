package frc.robot.factory;

import static frc.robot.Constants.*;

import frc.robot.subsystems.hood.HoodSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swerve.SwerveSubsystem;
import frc.robot.subsystems.turret.*;
import frc.robot.util.SimulationManager;

public final class TurretFactory {

    private TurretFactory() {}

    public static TurretSubsystem initialize(SwerveSubsystem swerve, ShooterSubsystem shooter, HoodSubsystem hood) {
        return switch (kCurrentMode) {
            case REAL -> initializeReal(swerve, shooter, hood);
            case SIM -> initializeSim(shooter, hood);
            case REPLAY -> initializeReplay(swerve, shooter, hood);
        };
    }

    private static TurretSubsystem initializeReal(
            SwerveSubsystem swerve, ShooterSubsystem shooter, HoodSubsystem hood) {
        return new TurretSubsystem(
                new TurretIOReal(),
                swerve::getPose,
                swerve::getChassisSpeeds,
                shooter::getCurrentVelocity,
                hood::getCurrentAngle);
    }

    private static TurretSubsystem initializeSim(ShooterSubsystem shooter, HoodSubsystem hood) {
        SimulationManager simulation = SimulationManager.getInstance();
        return new TurretSubsystem(
                new TurretIOSim(),
                simulation::getPose,
                simulation::getChassisSpeeds,
                shooter::getCurrentVelocity,
                hood::getCurrentAngle);
    }

    private static TurretSubsystem initializeReplay(
            SwerveSubsystem swerve, ShooterSubsystem shooter, HoodSubsystem hood) {
        return new TurretSubsystem(
                new TurretIO() {},
                swerve::getPose,
                swerve::getChassisSpeeds,
                shooter::getCurrentVelocity,
                hood::getCurrentAngle);
    }
}
