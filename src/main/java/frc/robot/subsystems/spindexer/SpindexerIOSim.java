package frc.robot.subsystems.spindexer;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.spindexer.SpindexerConstants.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.util.PowerSim;

public class SpindexerIOSim implements SpindexerIO {

    private final DCMotorSim spindexer;
    private final DCMotorSim feeder;

    public SpindexerIOSim() {
        spindexer = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), kMOI.in(KilogramSquareMeters), kGearRatio),
                DCMotor.getNEO(1));

        feeder = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(DCMotor.getNEO(1), kMOI.in(KilogramSquareMeters), kGearRatio),
                DCMotor.getNEO(1));
    }

    public void updateInputs(SpindexerInputs inputs) {
        spindexer.update(0.02);
        feeder.update(0.02);

        inputs.spindexerVelocity = spindexer.getAngularVelocityRPM();
        inputs.feederVelocity = feeder.getAngularVelocityRPM();

        inputs.spindexerCurrent = spindexer.getCurrentDrawAmps();
        inputs.feederCurrent = feeder.getCurrentDrawAmps();

        inputs.spindexerCurrent = spindexer.getInputVoltage();
        inputs.feederCurrent = feeder.getInputVoltage();
    }

    @Override
    public void setSpindexerSpeed(double speed) {
        double voltage = speed * PowerSim.getRailVoltage();
        spindexer.setInputVoltage(voltage);
    }

    @Override
    public void stopSpindexer() {
        spindexer.setInputVoltage(0.0);
    }

    @Override
    public void setFeederSpeed(double speed) {
        double voltage = speed * PowerSim.getRailVoltage();
        feeder.setInputVoltage(voltage);
    }

    @Override
    public void stopFeeder() {
        feeder.setInputVoltage(0.0);
    }
}
