package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOState {
        public double intakeRpm = 0.0;
        public boolean intakeUp = true;
        public double intakePositionCurrent = 0.0;
    }

    void updateState(IntakeIOState inputs);

    void clearPositionStickies();

    void run();

    void runReversed();

    void setPositionBrake();

    void setPositionCoast();

    void stop();

    void stopIntakePosition();

    void setIntakePositionSpeed(double speed);
}
