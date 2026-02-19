package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOReal implements ShooterIO {

    protected final TalonFX motor;

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    public ShooterIOReal() {
        motor = new TalonFX(kMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted =
                kInvertMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.CurrentLimits.SupplyCurrentLimit = 60.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;

        config.Slot0.kS = kS;
        config.Slot0.kV = kV;
        config.Slot0.kA = kA;

        motor.getConfigurator().apply(config);
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        var velocity = motor.getVelocity().asSupplier().get();
        var voltage = motor.getMotorVoltage().asSupplier().get();
        var current = motor.getSupplyCurrent().asSupplier().get();

        inputs.velocity = velocity;
        inputs.targetVelocity = RotationsPerSecond.of(velocityControl.Velocity);

        inputs.appliedVolts = voltage;
        inputs.outputCurrent = current;
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        motor.setControl(velocityControl.withVelocity(velocity.in(RotationsPerSecond)));
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageControl.withOutput(volts.in(Volts)));
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
