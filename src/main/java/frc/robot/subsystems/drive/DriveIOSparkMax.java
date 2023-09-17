package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax leftLeader;
    private final CANSparkMax leftFollower;
    private final CANSparkMax rightLeader;
    private final CANSparkMax rightFollower;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    protected DriveIOSparkMax() {
        leftLeader = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
        leftFollower = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

        rightLeader = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, CANSparkMaxLowLevel.MotorType.kBrushless);
        rightFollower = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, CANSparkMaxLowLevel.MotorType.kBrushless);

        leftLeader.setInverted(true);

        leftEncoder = leftLeader.getEncoder();
        rightEncoder = rightLeader.getEncoder();

        leftFollower.follow(leftLeader);
        rightFollower.follow(rightLeader);
    }

    @Override
    public void updateState(DriveIOState state) {
        state.brakeMode = CANSparkMax.IdleMode.kBrake == leftLeader.getIdleMode();
        state.encoderPositions = new double[] {leftEncoder.getPosition(), rightEncoder.getPosition()};
        state.wheelSpeeds = new DifferentialDrive.WheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    @Override
    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    @Override
    public void setBrakeMode() {
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);

        rightLeader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    @Override
    public void setCoastMode() {
        leftLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);

        rightLeader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    @Override
    public void setWheelSpeeds(DifferentialDrive.WheelSpeeds speeds) {
        leftLeader.setVoltage(speeds.left);
        rightLeader.setVoltage(speeds.right);
    }

    @Override
    public void stop() {
        leftLeader.stopMotor();
        rightLeader.stopMotor();
    }
}
