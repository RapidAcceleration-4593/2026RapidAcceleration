package frc.robot.util;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.*;
import static frc.robot.subsystems.hood.HoodConstants.kPhysicalOffset;
import static frc.robot.subsystems.swerve.SwerveConstants.kMapleSimConfig;
import static frc.robot.subsystems.vision.apriltag.AprilTagConstants.kFieldLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
import org.photonvision.simulation.VisionSystemSim;

public final class SimulationManager {

    private static SimulationManager instance;

    private final SwerveDriveSimulation swerveSim;
    private final SimulatedArena arena = SimulatedArena.getInstance();
    private final List<IPhysicsSim> components;

    private final VisionSystemSim visionSim;

    private boolean intakeExtended;

    private SimulationManager() {
        swerveSim = new SwerveDriveSimulation(kMapleSimConfig, new Pose2d());
        arena.addDriveTrainSimulation(swerveSim);
        components = new ArrayList<>();
        visionSim = new VisionSystemSim("main");
        visionSim.addAprilTags(kFieldLayout);
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
        swerveSim.setSimulationWorldPose(pose);
    }

    /** Retrieves the current simulated robot pose. */
    public Pose2d getPose() {
        return swerveSim.getSimulatedDriveTrainPose();
    }

    public void setChassisSpeeds(ChassisSpeeds speeds) {
        swerveSim.setAngularVelocity(speeds.omegaRadiansPerSecond);
        swerveSim.setLinearVelocity(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public ChassisSpeeds getChassisSpeeds() {
        return swerveSim.getDriveTrainSimulatedChassisSpeedsFieldRelative();
    }

    /** Resets the robot and field state for autonomous. */
    public void resetField() {
        setPose(FieldUtil.getInitialPose());
        arena.resetFieldForAuto();
    }

    /** Simulates an object being launched from the shooter mechanism. */
    private void launchProjectile(Angle turretAngle, Angle hoodAngle, AngularVelocity velocity) {
        Distance wheelRadius = Inches.of(2.0);
        LinearVelocity linearVelocity = MetersPerSecond.of(velocity.in(RadiansPerSecond) * wheelRadius.in(Meters));
        Rotation2d turretRotation = Rotation2d.fromRadians(turretAngle.in(Radians));

        RebuiltFuelOnFly projectile = new RebuiltFuelOnFly(
                getPose().getTranslation(),
                kPhysicalOffset.getTranslation(),
                getChassisSpeeds(),
                getPose().getRotation().plus(turretRotation),
                Inches.of(20.5),
                linearVelocity,
                Degrees.of(90.0).minus(hoodAngle));

        arena.addGamePieceProjectile(projectile);
    }

    public Command launchProjectileCommand(
            Supplier<Angle> turretAngle, Supplier<Angle> hoodAngle, Supplier<AngularVelocity> velocity) {
        return Commands.runOnce(() -> launchProjectile(turretAngle.get(), hoodAngle.get(), velocity.get()))
                .withTimeout(Seconds.of(0.4));
    }

    /** Run periodically during simulation. */
    public void periodic() {
        for (var component : components) {
            component.updatePlantSim();
        }

        arena.simulationPeriodic();
        visionSim.update(swerveSim.getSimulatedDriveTrainPose());

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
        return swerveSim;
    }

    public boolean isIntakeExtended() {
        return intakeExtended;
    }

    public void setIntakeExtended(boolean extended) {
        intakeExtended = extended;
    }

    public VisionSystemSim getVisionSim() {
        return visionSim;
    }
}
