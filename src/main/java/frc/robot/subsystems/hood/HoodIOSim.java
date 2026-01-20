package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.kMotorToEncoderGearing;

import com.revrobotics.sim.SparkMaxSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import frc.robot.util.PowerSim;

public class HoodIOSim extends HoodIOReal {
    private final SparkMaxSim motorSim;
    private final EncoderSim encoderSim;
    private final DIOSim lsSim;

    public HoodIOSim() {
        motorSim = new SparkMaxSim(motor, DCMotor.getNeo550(1));
        encoderSim = new EncoderSim(encoder);
        lsSim = new DIOSim(limitSwitch);
    }

    @Override
    public void updateInputs(HoodInputs inputs) {
        updateSimulation();
    }

    private void updateSimulation() {
        PowerSim.addCurrentDraw(motorSim.getMotorCurrent());
        encoderSim.setRate(motorSim.getVelocity() / kMotorToEncoderGearing);
    }
}
