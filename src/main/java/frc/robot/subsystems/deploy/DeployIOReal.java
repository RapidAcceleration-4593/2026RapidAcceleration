package frc.robot.subsystems.deploy;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.deploy.DeployConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
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
    private final Trigger lsTrigger;

    private Distance targetDistance = kMinimumDistance;

    public DeployIOReal() {
        deployConfig = new SparkMaxConfig();
        deployConfig.idleMode(IdleMode.kCoast).inverted(false);

        leftMotor = new SparkMax(kLeftMotorID, MotorType.kBrushless);
        leftMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        rightMotor = new SparkMax(kRightMotorID, MotorType.kBrushless);
        rightMotor.configure(deployConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = new Encoder(kEncoderChannelA, kEncoderChannelB);
        // TODO: Set encoder distance per pulse.

        limitswitch = new DigitalInput(kLimitSwitchChannel);

        lsTrigger = new Trigger(this::isLSPressed);
        lsTrigger.onTrue(Commands.runOnce(() -> {
            encoder.reset();
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
        leftMotor.setVoltage(0.0);
        rightMotor.setVoltage(0.0);
    }

    @Override
    public void setDistance(Distance distance) {
        targetDistance = distance;
    }

    @Override
    public void stopDeploy() {
        targetDistance = getDistance();
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    public Distance getDistance() {
        return Inches.of(encoder.getDistance());
    }

    private boolean atDistance() {
        return false; // TODO: Handle control loop.
    }

    private boolean isLSPressed() {
        return limitswitch.get();
    }
}
