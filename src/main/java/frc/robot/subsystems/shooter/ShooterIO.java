package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
    @AutoLog
    class ShooterIOState {
        public double leftRpm = 0.0;
        public double rightRpm = 0.0;
    }

    void updateState(ShooterIOState state);

    void setVoltages(double left, double right);

    void stop();
}
