package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {

    public static final int kIntakeMotorID = 9;

    public static final Voltage kIntakeVolts = Volts.of(10);
    public static final int kMaxCapacity = 35;

    public static final double kMotorToIntakeGearing = 4;
    public static final MomentOfInertia kIntakeMOI = KilogramSquareMeters.of(0.0004);

    public static final double kMotorVelocityIntakeThreshold = 0.1;
}
