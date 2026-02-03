package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.ExtraUnits.*;

import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {

    public static final int kIntakeMotorID = 0;

    public static final Voltage kIntakeVolts = Volts.of(0);
    public static final int kMaxCapacity = 35;

    public static final double kMotorToIntakeGearing = 4;
    public static final MomentOfInertia kIntakeMOI = PoundSquareInches.of(1.5);

    public static final double kMotorVelocityIntakeThreshold = 0.1;
}
