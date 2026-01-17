package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeConstants.*;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private final SparkMax intakeMotor;
    private final SparkMax deployMotor;

    private final DigitalInput deployLS;
    private final DigitalInput retractLS;

    public IntakeSubsystem() {
        intakeMotor = new SparkMax(kIntakeMotorID, MotorType.kBrushless);
        deployMotor = new SparkMax(kDeployMotorID, MotorType.kBrushless);

        deployLS = new DigitalInput(kDeployLS);
        retractLS = new DigitalInput(kRetractLS);
    }

    public void setIntakeSpeed(double speed) {
        intakeMotor.set(speed);
    }

    public void stopIntake() {
        intakeMotor.stopMotor();
    }

    public void setDeploySpeed(double speed) {
        deployMotor.set(speed);
    }

    public void stopDeploy() {
        deployMotor.stopMotor();
    }

    public boolean isDeployed() {
        return deployLS.get();
    }

    public boolean isRetracted() {
        return retractLS.get();
    }
}
