package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
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

    private final VelocityVoltage velocityControl = new VelocityVoltage(0);
    private final VoltageOut voltageControl = new VoltageOut(0);

    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Voltage> voltageSignal;
    private final StatusSignal<Current> currentSignal;

    private AngularVelocity targetVelocity = kZeroVelocity;

    public ShooterIOReal() {
        motor = new TalonFX(kMotorID);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput = new MotorOutputConfigs()
                .withInverted(kInvertMotor ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Coast);
        config.CurrentLimits =
                new CurrentLimitsConfigs().withSupplyCurrentLimit(60.0).withSupplyCurrentLimitEnable(true);
        config.Slot0 = new Slot0Configs().withKS(kS).withKV(kV).withKA(kA);

        motor.getConfigurator().apply(config);

        velocitySignal = motor.getVelocity();
        voltageSignal = motor.getMotorVoltage();
        currentSignal = motor.getSupplyCurrent();

        velocitySignal.setUpdateFrequency(100);
        voltageSignal.setUpdateFrequency(50);
        currentSignal.setUpdateFrequency(50);

        motor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ShooterInputs inputs) {
        velocitySignal.refresh();
        voltageSignal.refresh();
        currentSignal.refresh();

        inputs.velocity = velocitySignal.getValue();
        inputs.targetVelocity = targetVelocity;

        inputs.appliedVolts = voltageSignal.getValue();
        inputs.outputCurrent = currentSignal.getValue();
    }

    @Override
    public void setVelocity(AngularVelocity velocity) {
        targetVelocity = velocity;
        motor.setControl(velocityControl.withVelocity(velocity));
    }

    @Override
    public void setVoltage(Voltage volts) {
        motor.setControl(voltageControl.withOutput(volts));
    }

    @Override
    public void stop() {
        targetVelocity = kZeroVelocity;
        motor.stopMotor();
    }
}
