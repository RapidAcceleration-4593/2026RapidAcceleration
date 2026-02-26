package frc.robot.util.shooting;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.util.FieldUtil;

public final class ShotCalculatorConstants {

    public static final Angle kHoodVariability = Degrees.of(2.0);
    public static final LinearVelocity kShooterVariability = MetersPerSecond.of(0.5);
    public static final Translation3d kTarget = new Translation3d(
            FieldUtil.getTargetHubPose().getMeasureX(),
            FieldUtil.getTargetHubPose().getMeasureY(),
            Feet.of(5.0));
}
