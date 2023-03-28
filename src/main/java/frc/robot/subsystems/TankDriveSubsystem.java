package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Utilities.NAVX;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;

public class TankDriveSubsystem extends SubsystemBase{
    private static TankDriveSubsystem m_TankDriveSubsystem = new TankDriveSubsystem();

    private CANSparkMax primaryRightMotor;
    private CANSparkMax secondaryRightMotor;
    private CANSparkMax primaryLeftMotor;
    private CANSparkMax secondaryLeftMotor;

    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

    private RelativeEncoder leftRelativeEncoder;
    private RelativeEncoder rightRelativeEncoder;

    private DifferentialDriveOdometry m_Odometry;

    private double speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;

    public TankDriveSubsystem() {

        primaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
        primaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);

        primaryLeftMotor.setInverted(true);
        primaryRightMotor.setInverted(false);

        secondaryRightMotor.follow(primaryRightMotor);
        secondaryLeftMotor.follow(primaryLeftMotor);

        leftRelativeEncoder = primaryLeftMotor.getEncoder();
        rightRelativeEncoder = primaryRightMotor.getEncoder();

        leftRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        rightRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);

        m_Odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(NAVX.get().getAngle())),
                leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition());
    };

    @Override
    public void periodic() {
        if (Math.abs(NAVX.get().getRoll()) > 1) {
            m_Odometry.update(NAVX.get().getRotation2d(),
                    leftRelativeEncoder.getPosition() * Math.cos(Math.toRadians(NAVX.get().getRoll())),
                    rightRelativeEncoder.getPosition() * Math.cos(Math.toRadians(NAVX.get().getRoll())));
        } else {
            m_Odometry.update(NAVX.get().getRotation2d(),
                leftRelativeEncoder.getPosition(),
                rightRelativeEncoder.getPosition());
        }
    }

    // ================================Getters================================ \\

    public double[] getVoltages() {
        return new double[] { primaryLeftMotor.getAppliedOutput(), primaryRightMotor.getAppliedOutput()};
    }

    public static TankDriveSubsystem get() {
        return m_TankDriveSubsystem;
    }

    public Pose2d getRobotPose() {
        return m_Odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftRelativeEncoder.getVelocity(), rightRelativeEncoder.getVelocity());
    }

    public double[] getEncoderPositions() {
        return (new double[] { leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition() });
    }

    // ================================Setters================================ \\

    public void resetOdometry(Pose2d pose) {
        m_Odometry.resetPosition(NAVX.get().getRotation2d(), leftRelativeEncoder.getPosition(), rightRelativeEncoder.getPosition(), pose);
    }

    public void resetEncoders() {
        leftRelativeEncoder.setPosition(0);
        rightRelativeEncoder.setPosition(0);
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        primaryLeftMotor.setVoltage(leftVoltage);
        primaryRightMotor.setVoltage(rightVoltage);
    }

    public void setMotor(double leftSpeed, double rightSpeed) {
        primaryLeftMotor.set(leftSpeed);
        primaryRightMotor.set(rightSpeed);
    }

    public void SlowDrive() {
        speedCoefficient = .33;
    }

    public void UnslowDrive() {
        speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;
    }

    public void setBrake() {
        primaryLeftMotor.setIdleMode(IdleMode.kBrake);
        primaryRightMotor.setIdleMode(IdleMode.kBrake);
        secondaryLeftMotor.setIdleMode(IdleMode.kBrake);
        secondaryRightMotor.setIdleMode(IdleMode.kBrake);
    }

    public void setCoast() {
        primaryLeftMotor.setIdleMode(IdleMode.kCoast);
        primaryRightMotor.setIdleMode(IdleMode.kCoast);
        secondaryLeftMotor.setIdleMode(IdleMode.kCoast);
        secondaryRightMotor.setIdleMode(IdleMode.kCoast);
    }

    // ================================Commands================================ \\

    public class driveMotorCommand extends CommandBase {
        private DoubleSupplier m_rightSpeed;
        private DoubleSupplier m_leftSpeed;

        public driveMotorCommand(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed) {
            m_rightSpeed = rightSpeed;
            m_leftSpeed = leftSpeed;
            addRequirements(TankDriveSubsystem.this);
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            TankDriveSubsystem.this.setMotor(
                    adjustForDeadzone(m_leftSpeed.getAsDouble()) * speedCoefficient,
                    adjustForDeadzone(m_rightSpeed.getAsDouble()) * speedCoefficient);
        }

        private double adjustForDeadzone(double in) {
            if (Math.abs(in) < Constants.THUMBSTICK_DEADZONE) {
                return 0;
            }
            double sign = (in < 0) ? -Constants.MAX_DRIVETRAIN_SPEED : Constants.MAX_DRIVETRAIN_SPEED;
            return sign * Math.pow(in, 2);
        }
    }

    public class driveDistanceCommand extends CommandBase {
        private double m_rightSpeed;
        private double m_leftSpeed;
        private double[] m_initialPositions;
        private boolean hasReachedEnd = false;
        private double m_encoderLimit;

        public driveDistanceCommand(double leftSpeed, double rightSpeed, double encoder_limit) {
            m_encoderLimit = encoder_limit;
            m_leftSpeed = leftSpeed;
            m_rightSpeed = rightSpeed;
            addRequirements(TankDriveSubsystem.this);
        }
        
        @Override
        public void initialize() {
            m_initialPositions = TankDriveSubsystem.this.getEncoderPositions();
            TankDriveSubsystem.this.setMotor(m_leftSpeed, m_rightSpeed);
        }

        @Override
        public void execute() {
            double[] current_positions = TankDriveSubsystem.this.getEncoderPositions();
            double l_dif = (current_positions[0] - m_initialPositions[0]);
            double r_dif = (current_positions[1] - m_initialPositions[1]); 

            if (Math.abs(l_dif + r_dif) >= m_encoderLimit) {
                hasReachedEnd = true;
            }
        }
        
        @Override 
        public boolean isFinished() {
            return hasReachedEnd;
        }

        @Override 
        public void end(boolean isInterrupted){
            TankDriveSubsystem.this.setMotor(0, 0);
        }
    }
}