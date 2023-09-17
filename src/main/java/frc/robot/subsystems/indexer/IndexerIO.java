package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOState {
        public boolean beamBreakTriggered = false;
        public boolean useBeamBreak = true;
        public double beamBreakVoltage = 0.0;
        public double indexerRpm = 0.0;
    }

    /** Updates the set of loggable inputs. */
    void updateState(IndexerIOState inputs);

    void run();

    void runReversed();

    void stop();
}
