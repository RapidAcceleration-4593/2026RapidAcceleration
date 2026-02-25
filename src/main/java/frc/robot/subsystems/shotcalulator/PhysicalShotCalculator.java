package frc.robot.subsystems.shotcalulator;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.shooting.ProjectilePhysics;

import static frc.robot.subsystems.shotcalulator.PhysicalShotCalculatorConstants.*;

public class PhysicalShotCalculator extends SubsystemBase {
	Supplier<Pose2d> poseSupplier;
	ProjectilePhysics physics;
	ShotCalculation currentCalculation;

	public PhysicalShotCalculator(Supplier<Pose2d> poseSupplier) {
		this(poseSupplier, ProjectilePhysics.kFuelPhysics);
	}

	public PhysicalShotCalculator(Supplier<Pose2d> poseSupplier, ProjectilePhysics physics) {
		this.poseSupplier = poseSupplier;
		this.physics = physics;
	}
	
	@Override
	public void periodic() {
		currentCalculation = getBestShot();
	}

	public ShotCalculation getShotCalculation() {
		return currentCalculation;
	}

	private ShotCalculation getBestShot() {
		return null;
	}

	private ShotCalculation calculateBestShot(Pose2d position) {
		double distanceX = kTarget.toTranslation2d().minus(position.getTranslation()).getNorm();
		return null;
	}

	/**
	 * Calculates the distance between the {@code target} point and the closest point lying on the {@code trajectory}. Linearly interpolates between the {@link Translation2d}s describing the trajectory.
	 * @param trajectory The trajectory to be used in the calculation.
	 * @param target The target to be used in the calculation.
	 * @return The closest distance between the {@code trajectory} and the {@code target}.
	 */
	private Distance calculateTrajectoryError(List<Translation2d> trajectory, Translation2d target) {
		return null;
	}

	public record ShotCalculation(
		Angle hoodAngle,
		AngularVelocity shooterVelocity,
		boolean inRange
	) {}
}
