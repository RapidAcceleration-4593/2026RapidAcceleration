package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

    protected final TalonFX motor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
    private final VoltageOut voltageControl = new VoltageOut(0).withUpdateFreqHz(0);

	private final StatusSignal<AngularVelocity> velocitySignal;
	private final StatusSignal<Voltage> voltageSignal;
	private final StatusSignal<Current> currentSignal;

    public ShooterIOReal() {
        motor = new TalonFX(kMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted =
                kInvertMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Slot0 = kShooterGains;

        motor.getConfigurator().apply(config);

		velocitySignal = motor.getVelocity();
		voltageSignal = motor.getMotorVoltage();
		currentSignal = motor.getSupplyCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(100, velocitySignal, voltageSignal, currentSignal);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
		BaseStatusSignal.waitForAll(0.02, velocitySignal, voltageSignal, currentSignal);

        inputs.velocity = velocitySignal.getValue();
        inputs.targetVelocity = velocityControl.getVelocityMeasure();

        inputs.appliedVolts = voltageSignal.getValue();
        inputs.outputCurrent = currentSignal.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
