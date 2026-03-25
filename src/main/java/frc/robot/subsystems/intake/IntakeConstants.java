package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {

    public static final int kMotorID = 8;

    public static final boolean kInvertMotor = false;
    public static final int kMaxCapacity = 35;

    public static final Voltage kIntakeVolts = Volts.of(9.0);
    public static final double kIntakeGearing = 3.0 * (16.0 / 16.0);

    public static final MomentOfInertia kIntakeMOI = KilogramSquareMeters.of(0.0004);
}
