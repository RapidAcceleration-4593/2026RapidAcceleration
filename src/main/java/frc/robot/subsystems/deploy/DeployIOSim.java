package frc.robot.subsystems.deploy;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class DeployIOSim extends DeployIOReal {

    private final SparkMaxSim leftMotorSim;
    private final SparkMaxSim rightMotorSim;

    private final EncoderSim encoderSim;

    private final DCMotor gearbox;

    public DeployIOSim() {
        gearbox = DCMotor.getNeo550(2);

        leftMotorSim = new SparkMaxSim(leftMotor, gearbox);
        rightMotorSim = new SparkMaxSim(rightMotor, gearbox);

        encoderSim = new EncoderSim(encoder);

        // TODO: Add simulatable.
    }

    @Override
    public void updateInputs(DeployInputs inputs) {
        super.updateInputs(inputs);
    }
}
