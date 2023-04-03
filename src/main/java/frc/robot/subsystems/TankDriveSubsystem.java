package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private RelativeEncoder leftPrimaryRelativeEncoder;
    private RelativeEncoder rightPrimaryRelativeEncoder;
    private RelativeEncoder leftSecondaryRelativeEncoder;
    private RelativeEncoder rightSecondaryRelativeEncoder;

    private double previousLeftEncoder;
    private double previousRightEncoder;

    private DifferentialDriveOdometry m_Odometry;

    private double speedCoefficient = Constants.DRIVE_SPEED_COEFFICIENT;

    TankDriveSubsystem() {

        primaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
        primaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);
        secondaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);

        primaryLeftMotor.setInverted(true);
        primaryRightMotor.setInverted(false);

        secondaryRightMotor.follow(primaryRightMotor);
        secondaryLeftMotor.follow(primaryLeftMotor);

        leftPrimaryRelativeEncoder = primaryLeftMotor.getEncoder();
        rightPrimaryRelativeEncoder = primaryRightMotor.getEncoder();
        leftSecondaryRelativeEncoder = secondaryLeftMotor.getEncoder();
        rightSecondaryRelativeEncoder = secondaryRightMotor.getEncoder();

        leftPrimaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        rightPrimaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        leftSecondaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        rightSecondaryRelativeEncoder.setPositionConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE);
        /* 
        leftPrimaryRelativeEncoder.setVelocityConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE / 60);
        rightPrimaryRelativeEncoder.setVelocityConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE / 60);
        leftSecondaryRelativeEncoder.setVelocityConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE / 60);
        rightSecondaryRelativeEncoder.setVelocityConversionFactor(Constants.DRIVE_ROTATIONS_TO_DISTANCE / 60);
*/
        m_Odometry = new DifferentialDriveOdometry(new Rotation2d(Math.toRadians(NAVX.get().getAngle())),
                leftPrimaryRelativeEncoder.getPosition(), rightPrimaryRelativeEncoder.getPosition());
    };

    @Override
    public void periodic() {
        
        if (Math.abs(NAVX.get().getRoll()) > .75) {
            m_Odometry.update(
                NAVX.get().getRotation2d(),
                leftPrimaryRelativeEncoder.getPosition() - previousLeftEncoder * Math.cos(Math.toRadians(NAVX.get().getRoll())) + previousLeftEncoder,
                rightPrimaryRelativeEncoder.getPosition() - previousRightEncoder * Math.cos(Math.toRadians(NAVX.get().getRoll())) + previousRightEncoder);
        } else {
            m_Odometry.update(
                NAVX.get().getRotation2d(),
                leftPrimaryRelativeEncoder.getPosition(),
                rightPrimaryRelativeEncoder.getPosition()); 
            previousLeftEncoder = leftPrimaryRelativeEncoder.getPosition();
            previousRightEncoder = rightPrimaryRelativeEncoder.getPosition();
        }

        SmartDashboard.putNumberArray("Left Encoder Values", new double[] {
            leftPrimaryRelativeEncoder.getPosition(), 
            leftSecondaryRelativeEncoder.getPosition()
        });

        SmartDashboard.putNumberArray("Right Encoder Values", new double[] {
            rightPrimaryRelativeEncoder.getPosition(), 
            rightSecondaryRelativeEncoder.getPosition()
        });
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
        return new DifferentialDriveWheelSpeeds(leftPrimaryRelativeEncoder.getVelocity(), rightPrimaryRelativeEncoder.getVelocity());
    }

    public double[] getEncoderPositions() {
        return (new double[] { leftPrimaryRelativeEncoder.getPosition() / Constants.DRIVE_ROTATIONS_TO_DISTANCE, rightPrimaryRelativeEncoder.getPosition() / Constants.DRIVE_ROTATIONS_TO_DISTANCE });
    }

    // ================================Setters================================ \\

    public void resetOdometry(Pose2d pose) {
        leftPrimaryRelativeEncoder.setPosition(0);
        rightPrimaryRelativeEncoder.setPosition(0);
        m_Odometry.resetPosition(NAVX.get().getRotation2d(), leftPrimaryRelativeEncoder.getPosition(), rightPrimaryRelativeEncoder.getPosition(), pose);
    }

    public void resetEncoders() {
        leftPrimaryRelativeEncoder.setPosition(0);
        rightPrimaryRelativeEncoder.setPosition(0);
    }

    public void setVoltage(double leftVoltage, double rightVoltage) {
        SmartDashboard.putNumberArray("Set Voltages", new double[] {
            leftVoltage, rightVoltage
        });
        System.out.println(leftVoltage);
        System.out.println(rightVoltage);
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