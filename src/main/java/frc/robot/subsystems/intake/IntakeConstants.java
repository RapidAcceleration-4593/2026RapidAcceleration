package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {

    public static final int kMotorID = 1;

    public static final double kIntakeSpeed = 0.5;
    public static final int kMaxCapacity = 20;

    public static final Voltage kIntakeVolts = Volts.of(3.0);
}
