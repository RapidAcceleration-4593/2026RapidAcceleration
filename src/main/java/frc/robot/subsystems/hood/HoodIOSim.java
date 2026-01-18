package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class HoodIOSim implements HoodIO {

    private final SingleJointedArmSim hood;

    private final PIDController pid;

    private double targetAngle;
    private double appliedVolts;

    public HoodIOSim() {
        hood = new SingleJointedArmSim(
                DCMotor.getNeo550(1),
                kHoodGearRatio,
                kHoodMOI.in(KilogramSquareMeters),
                kHoodLength.in(Meters),
                kMinimumAngle.in(Radians),
                kMaximumAngle.in(Radians),
                false,
                kInitialAngle.in(Radians));

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(kToleranceAngle.in(Degrees));
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        hood.update(0.02);

        inputs.angle = Degrees.convertFrom(hood.getAngleRads(), Radians);
        inputs.appliedVolts = appliedVolts;
        inputs.limitswitch = hood.getAngleRads() <= kMinimumAngle.in(Radians);
    }

    @Override
    public void updateControl() {
        if (getLimitSwitch()) {
			pid.reset();

			if (targetAngle <= 0.0) {
				appliedVolts = 0.0;
				hood.setInputVoltage(0.0);
				return;
			}
		}

		appliedVolts = pid.calculate(getAngle(), targetAngle);
        hood.setInputVoltage(appliedVolts);
    }

    @Override
    public void setAngle(double degrees) {
        targetAngle = degrees;
    }

    @Override
    public double getAngle() {
        return hood.getAngleRads() * 180.0 / Math.PI;
    }

    @Override
    public boolean atAngle() {
        return pid.atSetpoint();
    }

    @Override
    public void stop() {
        targetAngle = getAngle();
        appliedVolts = 0.0;
        hood.setInputVoltage(0.0);
    }

    @Override
    public boolean getLimitSwitch() {
        return hood.getAngleRads() <= kMinimumAngle.in(Radians);
    }
}
