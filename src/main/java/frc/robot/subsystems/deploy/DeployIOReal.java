package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class DeployIOReal implements DeployIO {

    protected final SparkMax leftMotor;
    protected final SparkMax rightMotor;

    protected final Encoder encoder;
    protected final DigitalInput limitswitch;

    private final SparkMaxConfig deployConfig;
    private final SparkMaxConfig followerConfig;
    private final Trigger lsTrigger;

    private final PIDController controller;

    private Distance targetDistance = kMinimumDistance;

    public DeployIOReal() {
        leftMotor = new SparkMax(kLeftMotorID, MotorType.kBrushless);
        rightMotor = new SparkMax(kRightMotorID, MotorType.kBrushless);

        deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kCoast);

        followerConfig = new SparkMaxConfig();
        followerConfig.follow(leftMotor, true);

        leftMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        controller = new PIDController(kP, kI, kD);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        encoder.setDistancePerPulse(kDistancePerPulse);

        limitswitch = new DigitalInput(kLimitSwitchChannel);

        lsTrigger = new Trigger(this::isLSPressed);
        lsTrigger.onTrue(Commands.runOnce(() -> {
            encoder.reset();
            controller.reset();
        }));
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        inputs.distance = getDistance();
        inputs.targetDistance = targetDistance;

        inputs.atTargetDistance = atDistance();
        inputs.isLimitSwitchPressed = isLSPressed();

        inputs.leftAppliedVolts = Volts.of(leftMotor.getAppliedOutput() * leftMotor.getBusVoltage());
        inputs.rightAppliedVolts = Volts.of(rightMotor.getAppliedOutput() * rightMotor.getBusVoltage());

        inputs.leftOutputCurrent = Amps.of(leftMotor.getOutputCurrent());
        inputs.rightOutputCurrent = Amps.of(rightMotor.getOutputCurrent());
    }

    @Override
    public void updateControl() {
        double current = getDistance().in(Inches);
        double target = targetDistance.in(Inches);

        double volts = controller.calculate(current, target);
        volts = MathUtil.clamp(volts, -12.0, 12.0);

        leftMotor.setVoltage(volts);
    }

    @Override
    public void setDistance(Distance distance) {
        targetDistance = distance;
    }

    @Override
    public void stop() {
        targetDistance = getDistance();
        controller.reset();
        leftMotor.stopMotor();
    }

    public Distance getDistance() {
        return Inches.of(encoder.getDistance());
    }

    private boolean atDistance() {
        return controller.atSetpoint();
    }

    private boolean isLSPressed() {
        return limitswitch.get();
    }
}
