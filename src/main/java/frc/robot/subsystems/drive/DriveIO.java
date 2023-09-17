package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    class DriveIOState {
        public WheelSpeeds wheelSpeeds = new WheelSpeeds(0, 0); // TODO: Replace with double[]
        public boolean brakeMode = false;
        public double[] encoderPositions = new double[] {0, 0};
        public double speedCoefficient = 0.0;
    }

    void updateState(DriveIOState state);

    void resetEncoders();

    void setBrakeMode();

    void setCoastMode();

    void setWheelSpeeds(WheelSpeeds speeds);

    void stop();
}
