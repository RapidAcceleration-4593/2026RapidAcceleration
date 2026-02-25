package frc.robot.util.shooting;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public record ShotSolution(Angle turretAngle, Angle hoodAngle, AngularVelocity shooterVelocity, boolean valid) {}
