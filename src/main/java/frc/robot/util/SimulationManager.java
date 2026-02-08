package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.hood.HoodConstants.kPhysicalOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.MAPLESIM_CONFIG;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.Mode;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.motorsims.SimulatedBattery;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import org.littletonrobotics.junction.Logger;

public final class SimulationManager {

    private static SimulationManager instance;

    private final SwerveDriveSimulation simulation;
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final List<IPhysicsSim> components;

    private boolean intakeExtended;

    private SimulationManager() {
        simulation = new SwerveDriveSimulation(MAPLESIM_CONFIG, new Pose2d());
        arena.addDriveTrainSimulation(simulation);
        components = new ArrayList<>();
    }

    /** Retrieves the SimulationManager instance during simulation. */
    public static SimulationManager getInstance() {
        if (kCurrentMode != Mode.SIM) return null;
        if (instance == null) {
            instance = new SimulationManager();
        }
        return instance;
    }

    /** Sets the simulated robot pose. */
    public void setPose(Pose2d pose) {
        simulation.setSimulationWorldPose(pose);
    }

    /** Retrieves the current simulated robot pose. */
    public Pose2d getPose() {
        return simulation.getSimulatedDriveTrainPose();
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        simulation.setAngularVelocity(speeds.omegaRadiansPerSecond);
        simulation.setLinearVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    /** Resets the robot and field state for autonomous. */
    public void resetField() {
        Pose2d initialPose = FieldUtil.getInitialPose();

        setPose(initialPose);
        arena.resetFieldForAuto();
    }

    /** Simulates an object being launched from the shooter mechanism. */
    private void launchProjectile(Angle angle, AngularVelocity velocity) {
        double avgWheelRadiusMeters =
                Inches.of(1.25).in(Meters) / 2 + Inches.of(2.0).in(Meters) / 2;
        LinearVelocity linearVelocity = MetersPerSecond.of(velocity.in(RadiansPerSecond) * avgWheelRadiusMeters);

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                getPose().getTranslation(),
                kPhysicalOffset.getTranslation(),
                getChassisSpeeds(),
                getPose().getRotation(), // Plus turret rotation.
                Inches.of(20.5),
                linearVelocity,
                Degrees.of(90.0).minus(angle));

        arena.addGamePieceProjectile(projectile);
    }

    public Command launchProjectileCommand(Supplier<Angle> angle, Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> launchProjectile(angle.get(), velocity.get()))
                .withTimeout(Seconds.of(0.4));
    }

    /** Run periodically during simulation. */
    public void periodic() {
        for (var component : components) {
            component.updatePlantSim();
        }

        arena.simulationPeriodic();

        for (var component : components) {
            component.updateIOSim();
        }

        Logger.recordOutput("FieldSimulation/RobotPosition", getPose());
        Logger.recordOutput("FieldSimulation/Fuel", arena.getGamePiecesArrayByType("Fuel"));
    }

    /** Registers a subsystem physics simulation. */
    public void addSimulatable(IPhysicsSim sim) {
        components.add(sim);
        SimulatedBattery.addElectricalAppliances(sim::getCurrentDraw);
    }

    /** Retrieves the raw MapleSim drivetrain simulation. */
    public SwerveDriveSimulation getDriveSimulation() {
        return simulation;
    }

    public boolean isIntakeExtended() {
        return intakeExtended;
    }

    public void setIntakeExtended(boolean extended) {
        intakeExtended = extended;
    }
}
