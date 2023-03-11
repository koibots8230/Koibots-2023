// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;

public class TankDriveSubsystem extends SubsystemBase {

    private CANSparkMax primaryRightMotor;
    private CANSparkMax secondaryRightMotor;
    private CANSparkMax primaryLeftMotor;
    private CANSparkMax secondaryLeftMotor;

    private double speedCoefficient = 1;

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    DifferentialDrive drivetrain;
    DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds();

    private final RelativeEncoder primaryRightEncoder;
    private final RelativeEncoder primaryLeftEncoder;

    private final SparkMaxAbsoluteEncoder leftAbsoluteEncoder;
    private final SparkMaxAbsoluteEncoder rightAbsoluteEncoder;

    private DifferentialDriveOdometry m_Odometry;
    private Pose2d OdometryPose;

    public TankDriveSubsystem() {

        // Motors
        primaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_1, MotorType.kBrushless);
        primaryRightMotor.setInverted(true);

        primaryLeftMotor.getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        secondaryRightMotor = new CANSparkMax(Constants.RIGHT_DRIVE_MOTOR_2, MotorType.kBrushless);
        secondaryRightMotor.follow(primaryRightMotor);

        primaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_1, MotorType.kBrushless);

        secondaryLeftMotor = new CANSparkMax(Constants.LEFT_DRIVE_MOTOR_2, MotorType.kBrushless);
        secondaryLeftMotor.follow(primaryLeftMotor);

        // drivetrain = new DifferentialDrive(primaryLeftMotor, primaryRightMotor);

        // Encoders
        primaryRightEncoder = primaryRightMotor.getEncoder();
        primaryLeftEncoder = primaryLeftMotor.getEncoder();

        leftAbsoluteEncoder = primaryLeftMotor.getAbsoluteEncoder(Type.kDutyCycle);
        rightAbsoluteEncoder = primaryRightMotor.getAbsoluteEncoder(Type.kDutyCycle);

        primaryLeftEncoder.setPositionConversionFactor(Constants.LEFT_ENCODER_ROTATIONS_TO_DISTANCE);
        primaryRightEncoder.setPositionConversionFactor(Constants.RIGHT_ENCODER_ROTATIONS_TO_DISTANCE);

        primaryLeftEncoder.setVelocityConversionFactor(Constants.LEFT_ENCODER_ROTATIONS_TO_DISTANCE);
        primaryRightEncoder.setVelocityConversionFactor(Constants.RIGHT_ENCODER_ROTATIONS_TO_DISTANCE);

        m_Odometry = new DifferentialDriveOdometry(new Rotation2d(gyro.getYaw() + 180),
                primaryLeftEncoder.getPosition(), primaryRightEncoder.getPosition());
    }

    @Override
    public void periodic() {
        wheelSpeeds = new DifferentialDriveWheelSpeeds(primaryLeftEncoder.getVelocity(),
                primaryRightEncoder.getVelocity());
        OdometryPose = m_Odometry.update(
                new Rotation2d(gyro.getYaw() + 180),
                primaryLeftEncoder.getPosition(),
                primaryRightEncoder.getPosition());
        // This method will be called once per scheduler run
    }

    // ================================Getters================================

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

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return wheelSpeeds;
    }

    public double getLeftDriveSpeed() {
        return primaryLeftEncoder.getVelocity();
    }

    public double getRightDriveSpeed() {
        return primaryRightEncoder.getVelocity();
    }

    public Pose2d getOdometryPose() {
        return OdometryPose;
    }

    public SparkMaxPIDController getLeftPID() {
        return primaryLeftMotor.getPIDController();
    }

    public SparkMaxPIDController getRightPID() {
        return primaryRightMotor.getPIDController();
    }

    public double[] getEncoderPositions() {
        // get the in between of both encoders
        return (new double[] { primaryLeftEncoder.getPosition(), primaryRightEncoder.getPosition() });
    }

    // ================================Setters================================

    public void setOdometry(Pose2d pose) {
        primaryLeftEncoder.setPosition(0);
        primaryRightEncoder.setPosition(0);
        m_Odometry.resetPosition(
                gyro.getRotation2d(), primaryLeftEncoder.getPosition(), primaryRightEncoder.getPosition(), pose);
    }

    public void setMotor(double leftSpeed, double rightSpeed) {
        primaryLeftMotor.set(leftSpeed);
        primaryRightMotor.set(rightSpeed);
    }

    public void SlowDrive() {
        speedCoefficient = .33;
    }

    public void UnslowDrive() {
        speedCoefficient = 1;
    }

    public void setMotorVoltage(double leftVoltage, double rightVoltage) {
        primaryRightMotor.setVoltage(rightVoltage);
        primaryLeftMotor.setVoltage(leftVoltage);
        drivetrain.feed();
    }

    public SparkMaxAbsoluteEncoder getLeftAbsoluteEncoder() {
        return leftAbsoluteEncoder;
    }

    public SparkMaxAbsoluteEncoder getRightAbsoluteEncoder() {
        return rightAbsoluteEncoder;
    }

    // ================================Commands================================

    public class driveMotorCommand extends CommandBase {
        private DoubleSupplier m_rightSpeed;
        private DoubleSupplier m_leftSpeed;
        private TankDriveSubsystem m_DriveSubsystem;

        public driveMotorCommand(DoubleSupplier rightSpeed, DoubleSupplier leftSpeed, TankDriveSubsystem subsystem) {
            m_rightSpeed = rightSpeed;
            m_leftSpeed = leftSpeed;
            m_DriveSubsystem = subsystem;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {

        }

        @Override
        public void execute() {
            m_DriveSubsystem.setMotor(
                    adjustForDeadzone(m_leftSpeed.getAsDouble()) * speedCoefficient,
                    adjustForDeadzone(m_rightSpeed.getAsDouble()) * speedCoefficient);
        }

        private double adjustForDeadzone(double in) {
            if (Math.abs(in) < Constants.DEADZONE) {
                return 0;
            }
            double sign = (in < 0) ? -Constants.MAX_DRIVETRAIN_SPEED : Constants.MAX_DRIVETRAIN_SPEED;
            return sign * (in * in);
        }
    }

    public class driveDistanceCommand extends CommandBase {
        private double m_rightSpeed;
        private double m_leftSpeed;
        private SparkMaxPIDController m_rightPID;
        private SparkMaxPIDController m_leftPID;
        private TankDriveSubsystem m_DriveSubsystem;
        private double[] m_initialPositions;
        private boolean hasReachedEnd;
        private double m_encoderLimit;

        // Direction should be from -1 to 1 to indicate direction; 0 is Balanced, -1 is
        // full left, 1 is full right
        public driveDistanceCommand(double leftSpeed, double rightSpeed, double encoder_limit,
                TankDriveSubsystem subsystem) {
            m_DriveSubsystem = subsystem;
            m_encoderLimit = encoder_limit;
            m_leftSpeed = leftSpeed;
            m_rightSpeed = rightSpeed;
            addRequirements(subsystem);
        }

        @Override
        public void initialize() {
        }

        @Override
        public void execute() {
            m_leftPID.setReference((-m_leftSpeed), CANSparkMax.ControlType.kDutyCycle);
            m_rightPID.setReference((-m_rightSpeed), CANSparkMax.ControlType.kDutyCycle);

            // End Check
            double[] current_positions = m_DriveSubsystem.getEncoderPositions();
            double l_dif = (current_positions[0] - m_initialPositions[0]);
            double r_dif = (current_positions[1] - m_initialPositions[1]);

            if (Math.abs(l_dif + r_dif) >= m_encoderLimit) {
                System.out.println("Reached end condition for DriveDistance");
                hasReachedEnd = true;
            }
        }
        
        public void PPDrive(BiConsumer<Double, Double> speeds) {
            
        }

        @Override 
        public boolean isFinished() {
            return hasReachedEnd;
        }

        @Override
        public void end(boolean isInterrupted) {
            m_leftPID.setReference(0, CANSparkMax.ControlType.kDutyCycle);
            m_rightPID.setReference(0, CANSparkMax.ControlType.kDutyCycle);
        }
    }
}