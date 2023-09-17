package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.Constants;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless;

public class IndexerIOSparkMax implements IndexerIO {
    private final CANSparkMax motor;
    private final AnalogInput beamBreak;
    private final RelativeEncoder encoder;

    public IndexerIOSparkMax() {
        motor = new CANSparkMax(Constants.INDEXER_MOTOR, kBrushless);
        beamBreak = new AnalogInput(Constants.BEAM_BREAK);
        encoder = motor.getEncoder();
    }

    @Override
    public void updateState(IndexerIOState state) {
        state.beamBreakTriggered = beamBreakTriggered();
        state.beamBreakVoltage = beamBreak.getVoltage();
        state.indexerRpm = encoder.getVelocity();
    }

    private boolean beamBreakTriggered() {
        return beamBreak.getVoltage() < Constants.SENSOR_TRIGGERED; //TODO: Is this meant to be a less than?
    }

    @Override
    public void run() {
        motor.setVoltage(Constants.BELT_RUNNING_SPEED);
    }

    @Override
    public void runReversed() {
        motor.setVoltage(Constants.BELT_REVERSE_SPEED);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }
}
